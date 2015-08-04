// Copyright 2015 Native Client Authors.

uniform mat4 u_mvp;
attribute vec4 vertex;
attribute vec4 normal;
attribute vec4 color;
void main() { gl_Position = u_mvp * vec4(vertex.xyz, 1.0); }
