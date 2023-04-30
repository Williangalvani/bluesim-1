/*
Realistic Water Shader for GODOT 3.1.1

Copyright (c) 2019 UnionBytes, Achim Menzel (alias AiYori)

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the \"Software\"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
OR OTHER DEALINGS IN THE SOFTWARE.

-- UnionBytes <https://www.unionbytes.de/>
-- YouTube: <https://www.youtube.com/user/UnionBytes>
*/


// For this shader min. GODOT 3.1.1 is required, because 3.1 has a depth buffer bug!
shader_type 	spatial;
render_mode 	cull_back,diffuse_burley,specular_disabled, blend_mix;


// Wave settings:
uniform float	wave_speed		 = 0.5; // Speed scale for the waves
uniform vec4	wave_a			 = vec4(1.0, 1.0, 0.35, 3.0); 	// xy = Direction, z = Steepness, w = Length
uniform	vec4	wave_b			 = vec4(1.0, 0.6, 0.30, 1.55);	// xy = Direction, z = Steepness, w = Length
uniform	vec4	wave_c			 = vec4(1.0, 1.3, 0.25, 0.9); 	// xy = Direction, z = Steepness, w = Length

// Surface settings:
uniform vec2 	sampler_scale 	 = vec2(0.25, 0.25); 			// Scale for the sampler
uniform vec2	sampler_direction= vec2(0.05, 0.04); 			// Direction and speed for the sampler offset

uniform sampler2D uv_sampler : hint_anisotropy; 						// UV motion sampler for shifting the normalmap
uniform vec2 	uv_sampler_scale = vec2(0.25, 0.25); 			// UV sampler scale
uniform float 	uv_sampler_strength = 0.04; 					// UV shifting strength

uniform sampler2D normalmap_a_sampler : hint_normal;			// Normalmap sampler A
uniform sampler2D normalmap_b_sampler : hint_normal;			// Normalmap sampler B

uniform sampler2D foam_sampler : hint_default_black;					// Foam sampler
uniform float 	foam_level 		 = 0.5;							// Foam level -> distance from the object (0.0 - 0.5)
uniform sampler2D SCREEN_TEXTURE : hint_screen_texture, filter_linear_mipmap;
// Volume settings:
uniform float 	refraction 		 = 0.075;						// Refraction of the water
uniform float	alpha_factor 	 = 1.0;

uniform vec4 	ambient_color : source_color;						// Color for deep places in the water, medium to dark blue
uniform vec4 	color_shallow : source_color;						// Color for lower places in the water, bright blue - green
uniform vec4 	color_deep : source_color;						// Color for lower places in the water, bright blue - green

// Projector for the water caustics:
uniform mat4	projector;										// Projector matrix, mostly the matric of the sun / directlight
uniform sampler2DArray caustic_sampler : hint_default_black;			// Caustic sampler, (Texture array with 16 Textures for the animation)


// Vertex -> Fragment:
varying float 	vertex_height;									// Height of the water surface
varying vec3 	vertex_normal;									// Vertex normal -> Needed for refraction calculation
varying vec3 	vertex_binormal;								// Vertex binormal -> Needed for refraction calculation
varying vec3 	vertex_tangent;									// Vertex tangent -> Needed for refraction calculation

varying mat4 	inv_mvp; 										// Inverse ModelViewProjection matrix -> Needed for caustic projection


// Wave function:
vec4 wave(vec4 parameter, vec2 position, float time, inout vec3 tangent, inout vec3 binormal)
{
	float	wave_steepness	 = parameter.z;
	float	wave_length		 = parameter.w;

	float	k				 = 2.0 * 3.14159265359 / wave_length;
	float 	c 				 = sqrt(9.8 / k);
	vec2	d				 = normalize(parameter.xy);
	float 	f 				 = k * (dot(d, position) - c * time);
	float 	a				 = wave_steepness / k;
	
			tangent			+= normalize(vec3(1.0-d.x * d.x * (wave_steepness * sin(f)), d.x * (wave_steepness * cos(f)), -d.x * d.y * (wave_steepness * sin(f))));
			binormal		+= normalize(vec3(-d.x * d.y * (wave_steepness * sin(f)), d.y * (wave_steepness * cos(f)), 1.0-d.y * d.y * (wave_steepness * sin(f))));

	return vec4(d.x * (a * cos(f)), a * sin(f) * 0.25, d.y * (a * cos(f)), 0.0);
}


// Vertex shader:
void vertex()
{
	float	time			 = TIME * wave_speed;
	
	vec4	vertex			 = vec4(VERTEX, 1.0);
	vec3	vertex_position  = (MODEL_MATRIX * vertex).xyz;
	
			vertex_tangent 	 = vec3(0.0, 0.0, 0.0);
		 	vertex_binormal  = vec3(0.0, 0.0, 0.0);
	
			vertex 			+= wave(wave_a, vertex_position.xz, time, vertex_tangent, vertex_binormal);
			vertex 			+= wave(wave_b, vertex_position.xz, time, vertex_tangent, vertex_binormal);
			vertex 			+= wave(wave_c, vertex_position.xz, time, vertex_tangent, vertex_binormal);
	
			vertex_position  = vertex.xyz;
	
			vertex_height	 = (PROJECTION_MATRIX * MODELVIEW_MATRIX * vertex).z;
	
			TANGENT			 = vertex_tangent;
			BINORMAL		 = vertex_binormal;
			vertex_normal	 = normalize(cross(vertex_binormal, vertex_tangent));
			NORMAL			 = vertex_normal;
		
			UV				 = vertex.xz * sampler_scale;
	
			VERTEX			 = vertex.xyz;
			
			inv_mvp = inverse(PROJECTION_MATRIX * MODELVIEW_MATRIX);
}


// Fragment shader:
void fragment()
{
	// Calculation of the UV with the UV motion sampler
	vec2	uv_offset 					 = sampler_direction * TIME;
	vec2 	uv_sampler_uv 				 = UV * uv_sampler_scale + uv_offset;
	vec2	uv_sampler_uv_offset 		 = uv_sampler_strength * texture(uv_sampler, uv_sampler_uv).rg * 2.0 - 1.0;
	vec2 	uv 							 = UV + uv_sampler_uv_offset;
	
	// Normalmap:
	vec3 	normalmap					 = texture(normalmap_a_sampler, uv - uv_offset*2.0).rgb * 0.75;		// 75 % sampler A
			normalmap 					+= texture(normalmap_b_sampler, uv + uv_offset).rgb * 0.25;			// 25 % sampler B
	
	// Refraction UV:
	vec3	ref_normalmap				 = normalmap * 2.0 -1.0;
			ref_normalmap				 = normalize(vertex_tangent*ref_normalmap.x + vertex_binormal*ref_normalmap.y + vertex_normal*ref_normalmap.z);
	vec2 	ref_uv						 = SCREEN_UV + (ref_normalmap.xy * refraction) / vertex_height;
	
	// Ground depth:
	float	depth_blend_pow				 = 0.0;

	// Ground color:
	vec3 	screen_color 				 = textureLod(SCREEN_TEXTURE, ref_uv, depth_blend_pow * 2.5).rgb;
	
	vec3 	dye_color 					 = mix(color_shallow.rgb, ambient_color.rgb, depth_blend_pow);
	vec3	color 						 = mix(screen_color*dye_color, dye_color*0.25, depth_blend_pow*0.5);
	
	
		// Set all values:
	vec3 world_view = (INV_VIEW_MATRIX * vec4(VIEW, 0.0)).xyz;
	float transparency_map = 1.0 - step(0.25, pow(max(0.0, dot(ref_normalmap, world_view)), 2.0));
	ALBEDO = color_deep.rgb;
	// Calculate the brightness of the original color
	float brightness = dot(color.rgb, vec3(0.299, 0.587, 0.114));
	// Blend the deep_color with the original color based on brightness
	EMISSION = mix(color_deep.rgb, color_shallow.rgb * (1.0 - transparency_map), (1.0 - transparency_map));
	METALLIC = 1.0;
	ROUGHNESS = 0.9;
	SPECULAR = 0.0; // + depth_blend_pow * 0.4;
	NORMAL_MAP = normalmap;
	NORMAL_MAP_DEPTH = 1.25;
	ALPHA = clamp(alpha_factor + 1.0 - transparency_map, 0.3, 1.0);
}
