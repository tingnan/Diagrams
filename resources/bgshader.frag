#ifdef GL_ES
precision mediump float;
#endif

uniform float time;
uniform vec2 resolution;

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

void main() {
	vec2 uv = gl_FragCoord.xy / resolution.xy;
	float frag_color = PatternCompose(uv) * exp(0.5 * length_square(uv));
	gl_FragColor = vec4(0, 0.6 * frag_color * frag_color, frag_color, 1.0);
}
