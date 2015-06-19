attribute vec3 vertexPosition;
attribute vec3 vertexColor;
void main()
{
	gl_Position = vec4(vertexPosition, 1.0);
	gl_Position.w = 1.0;
	fragmentColor = vertexColor;
}
