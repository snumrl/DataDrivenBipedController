uniform sampler2DShadow		shadowMap;
uniform sampler2D			decalTex;
varying vec3				N, L;
varying vec4				q;

void main(void)
{
	vec3 R = -normalize(reflect(L, N));
	vec4 ambient = gl_FrontLightProduct[0].ambient;
	vec4 diffuse = gl_FrontLightProduct[0].diffuse * max(dot(N, L), 0.0);
	float shadow = shadow2D(shadowMap, 0.5 * (q.xyz / q.w + 1.0)).r;
	gl_FragColor = texture2D(decalTex, gl_TexCoord[0].st) * (ambient + (0.5 + 0.5 * shadow) * diffuse);
	//gl_FragColor = (ambient + (0.5 + 0.5 * shadow) * diffuse);
}
