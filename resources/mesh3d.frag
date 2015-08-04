// Copyright 2015 Native Client Authors.

#ifdef GL_ES
precision mediump float;
#endif

uniform float time;
varying vec4 output_color;
varying vec4 normal_direction;
varying vec2 uv;

float length_square(vec2 p) {
    return dot(p, p);
}

float noise(vec2 p) {
    return fract(cos(p.x) * sin(p.y) * 31.0 + p.y - p.x * 23.0);
}

float Pattern(vec2 p) {
	float d = 1e9;
	for (int x = 0; x <= 1; ++x) {
	for (int y = 0; y <= 1; ++y)
	{
    	    vec2 tp = floor(p) + vec2(x, y);
    	    float tmp = length_square(p - tp + 0.5 * noise(tp));
	    d = min(d, tmp);
	}
	}
	return 1.2 * exp(-2.0 * abs(2.0 * d - 1.0));
}

float PatternCompose(vec2 p) {
	return sqrt(sqrt(Pattern(17.0 * p - 0.5 * time) * Pattern(5.0 * p + 0.2 * time) * sqrt(sqrt(Pattern(-5.0 * p)))));
}


const vec3 light_direction = vec3(1.0, 1.0, 1.0);
void main( void ) {
  vec3 light_norm = normalize(light_direction);
  float cos_theta = clamp(dot(light_norm, normal_direction.xyz), 0.0, 1.0);
  float light_color = cos_theta * PatternCompose(uv);
  light_color *= 3.0 * light_color;
  vec4 color = output_color.aaaa;
  gl_FragColor =  color * vec4(light_color, 1.5 * light_color, light_color, 1.0);
}
