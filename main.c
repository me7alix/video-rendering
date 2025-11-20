#include <math.h>
#include <stdio.h>
#define LINALG_IMPLEMENTATION
#include "linalg.h"

/* ================= */
/* "CPU shader" part */
/* ================= */

Vec3 scene_color;
float scene_dist(Vec3 p) {
	Vec3 w = p;
	float m = v3dot(w, w);

	Vec4 trap = v4(fabsf(w.x), fabsf(w.y), fabsf(w.z), m);
	float dz = 1.0;

	for (int i = 0; i < 4; i++) {
		float m2 = m * m;
		float m4 = m2 * m2;
		dz = 8.0 * sqrtf(m4 * m2 * m) * dz + 1.0;

		float x = w.x; float x2 = x*x; float x4 = x2*x2;
		float y = w.y; float y2 = y*y; float y4 = y2*y2;
		float z = w.z; float z2 = z*z; float z4 = z2*z2;

		float k3 = x2 + z2;
		float k2 = 1.0/sqrtf(k3*k3*k3*k3*k3*k3*k3);
		float k1 = x4 + y4 + z4 - 6.0*y2*z2 - 6.0*x2*y2 + 2.0*z2*x2;
		float k4 = x2 - y2 + z2;

		w.x = p.x +  64.0*x*y*z*(x2-z2)*k4*(x4-6.0*x2*z2+z4)*k1*k2;
		w.y = p.y + -16.0*y2*k3*k4*k4 + k1*k1;
		w.z = p.z +  -8.0*y*k4*(x4*x4 - 28.0*x4*x2*z2 + 70.0*x4*z4 - 28.0*x2*z2*z4 + z4*z4)*k1*k2;

		trap = v4(fminf(fabsf(w.x), trap.x), fminf(fabsf(w.y), trap.y),
			fminf(fabsf(w.z), trap.z), fminf(m, trap.w));

		m = v3dot(w, w);
		if (m > 256.0)
			break;
	}

	scene_color = v3(fminf(m, 1.0), fminf(trap.y, 1.0), fminf(trap.z, 1.0));

	float res = 0.25*logf(m)*sqrtf(m)/dz;
	if (!isfinite(res)) return 256.0;
	return res;
}

Vec3 scene_norm(Vec3 p){
	Vec2 e = {0.005, 0};
	float d = scene_dist(p);
	return v3norm(v3(
		d - scene_dist(v3sub(p, v3(e.x, e.y, e.y))),
		d - scene_dist(v3sub(p, v3(e.y, e.x, e.y))),
		d - scene_dist(v3sub(p, v3(e.y, e.y, e.x)))
	));
}

void ray_marching(Vec3 ro, Vec3 rd, int *out_hit, Vec3 *out_p) {
	*out_hit = 0;
	float dO = 0;

	for (;;) {
		Vec3 p = v3add(ro, v3scale(rd, dO));
		float sc = scene_dist(p);
		dO += sc;
		if (dO > 15.0) break;
		else if (sc < 0.01) {
			*out_p = p;
			*out_hit = 1;
			break;
		}
	}
}

Vec3 pixel_prog(int w, int h, int x, int y, float t) {
	Vec2 uv = {(x - 0.5 * w) / h, (y - 0.5 * h) / h};
	Vec3 ro = {0, 1.2, -1 - 2 * (sinf(t) / 2.0 + 0.5)};
	Vec3 rd = v3norm(v3(uv.x, -uv.y, 1));

	vrot(rd.y, rd.z, 0.11 * PI);
	vrot(rd.x, rd.z, t);
	vrot(ro.x, ro.z, t);

	int hit; Vec3 p;
	ray_marching(ro, rd, &hit, &p);
	if (!hit) return v3(0);

	Vec3 light_dir = v3norm(v3(cosf(PI), 1.0, sinf(PI)));
	float light = fmaxf(v3dot(scene_norm(p), light_dir), 0.08);
	Vec3 clr = v3scale(scene_color, light);

	return clr;
}

/* ============= */
/* Renderer part */
/* ============= */

typedef struct {
	unsigned char y, cb, cr;
} YCbCr;

YCbCr rgb_to_ycbcr(int r, int g, int b) {
	return (YCbCr){
		.y = 16 + (65.738 * r + 129.057 * g + 25.064 * b) / 256,
		.cb = 128 + (-37.945 * r - 74.494 * g + 112.439 * b) / 256,
		.cr = 128 + (112.439 * r - 94.154 * g - 18.285 * b) / 256,
	};
}

YCbCr vec3_to_ycbcr(Vec3 col) {
	return (YCbCr){
		.y  = 16 + (65.738f * col.x + 129.057f * col.y + 25.064f * col.z),
		.cb = 128 + (-37.945f * col.x - 74.494f * col.y + 112.439f * col.z),
		.cr = 128 + (112.439f * col.x - 94.154f * col.y - 18.285f * col.z)
	};
}

int main() {
	const int w = 16 * 40;
	const int h = 9 * 40;
	const int fps = 30;
	const int frames_num = fps * 6;
	unsigned char fr_y[w * h], fr_cr[w * h], fr_cb[w * h];

	FILE *f = fopen("output.y4m", "wb");
	fprintf(f, "YUV4MPEG2 W%d H%d F%d:1 Ip A1:1 C444\n", w, h, fps);

	for (int fr = 0; fr < frames_num; fr++) {
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				Vec3 v3col = pixel_prog(w, h, x, y, (float)fr / fps);
				YCbCr col = vec3_to_ycbcr(v3col);
				fr_y [y * w + x] = col.y;
				fr_cb[y * w + x] = col.cb;
				fr_cr[y * w + x] = col.cr;
			}
		}

		fprintf(f, "FRAME\n");
		fwrite(fr_y,  1, w*h, f);
		fwrite(fr_cb, 1, w*h, f);
		fwrite(fr_cr, 1, w*h, f);

		float progress = (float)(fr + 1) / frames_num * 100;
		if ((fr + 1) % 10 == 0)
			printf("%3.0f%% rendering...\n", progress);
	}

	fclose(f);
	printf("Generated output.y4m\n");

	return 0;
}
