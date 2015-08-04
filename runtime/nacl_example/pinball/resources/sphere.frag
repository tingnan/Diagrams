// Copyright 2015 Native Client Authors.

#ifdef GL_ES
precision mediump float;
#endif

varying vec2 uv;
varying float radius;
varying vec4 output_color;

const vec3 light_direction_1 = vec3(1.0, 1.0, 1.0);
void main( void ) {
  float dist = length(uv);
  float frag_color = 1.0 - dist;
  if (frag_color < 0.0) { frag_color = 0.0;}
  if (dist > 1.0) {
    discard;
  } else {
    gl_FragColor = output_color * vec4(0, frag_color, frag_color, 1);
  }
}
