// Renders a BlueSim level top-down into map tiles for Cockpit's map widget.
//
//   node tools/render_map_tiles.mjs [--level reef] [--lat 33.810313] [--lon -118.393867] [--zoom 22] [--minzoom 15]
//
// Serves the web export in export/web, runs it in headless Chrome with ?maptiles (see tools/map_tiles.gd), and
// writes export/web/tiles/<level>/{z}/{x}/{y}.jpg, served next to BlueSim as an XYZ URL, plus
// export/map-tiles/<level>.zip for Cockpit's custom tile provider import. The lat/lon are the SITL home.
// CHROME overrides the browser binary.
import { spawn } from 'node:child_process'
import { mkdirSync, mkdtempSync, readFileSync, renameSync, rmSync, writeFileSync } from 'node:fs'
import { createServer } from 'node:http'
import { tmpdir } from 'node:os'
import { dirname, extname, join, normalize, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'
import { parseArgs } from 'node:util'

const { values: args } = parseArgs({
  options: {
    level: { type: 'string', default: 'reef' },
    lat: { type: 'string', default: '33.810313' },
    lon: { type: 'string', default: '-118.393867' },
    zoom: { type: 'string', default: '22' },
    minzoom: { type: 'string', default: '15' },
    timeout: { type: 'string', default: '300' },
  },
})
const root = resolve(dirname(fileURLToPath(import.meta.url)), '..')
const webRoot = join(root, 'export/web')
const tilesDir = join(webRoot, 'tiles', args.level)
const zipPath = join(root, 'export/map-tiles', `${args.level}.zip`)
// written beside the final directory and swapped in at the end, so a failed run never leaves partial tiles
const stagingDir = `${tilesDir}.partial`
const stagingZip = `${zipPath}.partial`

const mimeTypes = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.pck': 'application/octet-stream', '.png': 'image/png' }
const server = createServer((request, response) => {
  const path = normalize(decodeURIComponent(new URL(request.url, 'http://x').pathname)).replace(/^(\.\.[/\\])+/, '')
  try {
    const body = readFileSync(join(webRoot, path === '/' ? 'index.html' : path))
    response.writeHead(200, { 'Content-Type': mimeTypes[extname(path)] ?? 'application/octet-stream' })
    response.end(body)
  } catch {
    response.writeHead(404).end()
  }
})
await new Promise((ready) => server.listen(0, '127.0.0.1', ready))

const sleep = (ms) => new Promise((done) => setTimeout(done, ms))
const port = 9335
const chromeProfile = mkdtempSync(join(tmpdir(), 'bluesim-map-tiles-'))
const chrome = spawn(process.env.CHROME ?? 'google-chrome-stable', [
  '--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${chromeProfile}`,
  '--ignore-gpu-blocklist', '--enable-gpu', '--use-angle=vulkan', '--enable-features=Vulkan', 'about:blank',
], { stdio: 'ignore' })
let chromeError
chrome.on('error', (error) => (chromeError = error))

let exitCode = 1
try {
  let targets
  for (let i = 0; i < 50 && !targets && !chromeError; i++) {
    await sleep(200)
    targets = await fetch(`http://127.0.0.1:${port}/json`).then((r) => r.json()).catch(() => undefined)
  }
  if (!targets) throw new Error(`Chrome did not start${chromeError ? `: ${chromeError.message}` : ''}`)
  const ws = new WebSocket(targets.find((t) => t.type === 'page').webSocketDebuggerUrl)
  await new Promise((open, fail) => { ws.addEventListener('open', open); ws.addEventListener('error', fail) })
  let id = 0
  const pending = new Map()
  ws.addEventListener('message', (event) => {
    const message = JSON.parse(event.data)
    pending.get(message.id)?.(message)
    pending.delete(message.id)
  })
  const send = (method, params = {}) => new Promise((done) => {
    pending.set(++id, done)
    ws.send(JSON.stringify({ id, method, params }))
  })
  const evaluate = async (expression) => (await send('Runtime.evaluate', { expression, returnByValue: true })).result?.result?.value

  const query = new URLSearchParams({ maptiles: '', level: args.level, lat: args.lat, lon: args.lon, zoom: args.zoom, minzoom: args.minzoom })
  await send('Page.navigate', { url: `http://127.0.0.1:${server.address().port}/index.html?${query}` })

  rmSync(stagingDir, { recursive: true, force: true })
  let written = 0
  const deadline = Date.now() + Number(args.timeout) * 1000
  for (;;) {
    if (Date.now() > deadline) throw new Error(`timed out after ${written} tiles`)
    await sleep(500)
    const state = await evaluate('({ tiles: window.mapTiles?.splice(0) ?? [], done: !!window.mapTilesDone, error: window.mapTilesError })')
    if (state?.error) throw new Error(state.error)
    for (const [name, base64] of state?.tiles ?? []) {
      const file = join(stagingDir, name)
      mkdirSync(dirname(file), { recursive: true })
      writeFileSync(file, Buffer.from(base64, 'base64'))
      written++
    }
    if (state?.done) break
  }
  if (written === 0) throw new Error('the level rendered no tiles')

  rmSync(tilesDir, { recursive: true, force: true })
  renameSync(stagingDir, tilesDir)
  mkdirSync(dirname(zipPath), { recursive: true })
  rmSync(zipPath, { force: true })
  rmSync(stagingZip, { force: true })
  const zip = spawn('zip', ['-qr0', stagingZip, '.'], { cwd: tilesDir, stdio: 'inherit' })
  const zipCode = await new Promise((done, fail) => zip.on('close', done).on('error', fail))
  if (zipCode !== 0) throw new Error(`zip exited with ${zipCode}`)
  renameSync(stagingZip, zipPath)
  console.log(`${written} tiles in ${tilesDir}, archive ${zipPath}`)
  exitCode = 0
} catch (error) {
  console.error(`map tiles failed: ${error.message}`)
  rmSync(stagingDir, { recursive: true, force: true })
  rmSync(stagingZip, { force: true })
} finally {
  chrome.kill()
  server.close()
  rmSync(chromeProfile, { recursive: true, force: true })
}
process.exit(exitCode)
