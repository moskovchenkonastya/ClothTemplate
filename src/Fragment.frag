#version 330

uniform sampler2D diffuseTexture;

in vec2 fragmentTexCoord;
out vec4 fragColor;

void main(void)
{	
  // fragColor = vec4(0,1,0,1);
  fragColor = texture(diffuseTexture, fragmentTexCoord * 4);
}





