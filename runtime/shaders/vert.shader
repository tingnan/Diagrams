#version 330 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexColor;
uniform mat4 modelToEyeMat;

out vec3 fragmentColor;

void main()
{
	gl_Position = modelToEyeMat * vec4(vertexPosition, 1.0) * 0.002;
	gl_Position.w = 1.0;
	fragmentColor = vertexColor;
}
