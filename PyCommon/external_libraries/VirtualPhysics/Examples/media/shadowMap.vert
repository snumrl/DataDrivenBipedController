varying vec3		N, L;
varying vec4		q;
uniform mat4		T;

void main(void)
{
	vec4 p = gl_ModelViewMatrix * gl_Vertex;
	L = normalize(gl_LightSource[0].position.xyz - p.xyz);
	N = normalize(gl_NormalMatrix * gl_Normal);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;
	q = T * p;
}
