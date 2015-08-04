// Copyright 2015 Native Client Authors.

uniform mat4 u_mvp;
attribute vec4 vertex;
attribute vec4 normal;
attribute vec4 color;
varying vec4 output_color;
varying vec4 normal_direction;
varying vec2 uv;
void main() {
  gl_Position = u_mvp * vec4(vertex.xyz, 1.0);
  uv = gl_Position.xy / gl_Position.w;
  output_color = color;
  normal_direction = normal;
}
