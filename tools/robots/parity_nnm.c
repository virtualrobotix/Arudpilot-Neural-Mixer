// Author: Roberto Navoni, member of the ArduPilot Dev Team
// Contact: r.navoni74@gmail.com
// Developed by Roberto Navoni — DelphyAI LAB
// For information: r.navoni74@gmail.com
//
// Host parity harness: load an NNM1 file and run nnmixer_forward_int8() from
// tools/nnmixer_infer.c (the same file compiled into AP_NNMixer).
//
//   cc -O2 -I tools tools/robots/parity_nnm.c tools/nnmixer_infer.c -lm -o /tmp/parity_nnm
//   /tmp/parity_nnm policy.nnm obs.f32 act.f32      # obs: N*obs_dim float32, act: N*act_dim float32
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "nnmixer_infer.h"

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static uint8_t *read_file(const char *path, long *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(1); }
    fseek(f, 0, SEEK_END);
    *len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc((size_t)*len);
    if (fread(buf, 1, (size_t)*len, f) != (size_t)*len) { perror("read"); exit(1); }
    fclose(f);
    return buf;
}

static float *floats_at(const uint8_t *p, uint32_t n)
{
    float *out = malloc(n * sizeof(float));
    memcpy(out, p, n * sizeof(float));
    return out;
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: %s policy.nnm obs.f32 act.f32\n", argv[0]);
        return 2;
    }
    long len;
    uint8_t *blob = read_file(argv[1], &len);
    if (len < 28 || memcmp(blob, "NNM1", 4) != 0) { fprintf(stderr, "not NNM1\n"); return 1; }
    const uint8_t *p = blob + 20;
    nnmixer_policy_int8_t pol;
    memset(&pol, 0, sizeof(pol));
    pol.obs_dim = rd_u16(p); p += 2;
    pol.act_dim = rd_u16(p); p += 2;
    pol.n_layers = *p++;
    pol.flags = *p++;
    p += 2;
    if (pol.n_layers == 0 || pol.n_layers > NNMIXER_MAX_LAYERS) { fprintf(stderr, "bad layers\n"); return 1; }
    uint16_t *dims = malloc((pol.n_layers + 1) * sizeof(uint16_t));
    for (int i = 0; i <= pol.n_layers; i++) { dims[i] = rd_u16(p); p += 2; }
    uint8_t *act = malloc(pol.n_layers);
    memcpy(act, p, pol.n_layers); p += pol.n_layers;
    pol.dims = dims;
    pol.act = act;
    pol.obs_mean = floats_at(p, pol.obs_dim); p += pol.obs_dim * 4;
    pol.obs_std = floats_at(p, pol.obs_dim); p += pol.obs_dim * 4;
    pol.default_pose = floats_at(p, pol.act_dim); p += pol.act_dim * 4;
    const int8_t **W = malloc(pol.n_layers * sizeof(*W));
    const float **S = malloc(pol.n_layers * sizeof(*S));
    const float **B = malloc(pol.n_layers * sizeof(*B));
    for (int l = 0; l < pol.n_layers; l++) {
        uint32_t n_in = dims[l], n_out = dims[l + 1];
        S[l] = floats_at(p, n_out); p += n_out * 4;
        B[l] = floats_at(p, n_out); p += n_out * 4;
        W[l] = (const int8_t *)p; p += n_out * n_in;
    }
    if (p - blob != len) { fprintf(stderr, "size mismatch %ld vs %ld\n", (long)(p - blob), len); return 1; }
    pol.W = W;
    pol.w_scale = S;
    pol.b = B;

    long olen;
    uint8_t *obs = read_file(argv[2], &olen);
    long n = olen / (long)(pol.obs_dim * sizeof(float));
    float *out = malloc((size_t)n * pol.act_dim * sizeof(float));
    for (long k = 0; k < n; k++) {
        float o[512];
        memcpy(o, obs + k * pol.obs_dim * sizeof(float), pol.obs_dim * sizeof(float));
        if (nnmixer_forward_int8(&pol, o, out + k * pol.act_dim) != 0) { fprintf(stderr, "forward err\n"); return 1; }
    }
    FILE *f = fopen(argv[3], "wb");
    fwrite(out, sizeof(float), (size_t)n * pol.act_dim, f);
    fclose(f);
    printf("ok %ld samples obs=%u act=%u layers=%u\n", n, pol.obs_dim, pol.act_dim, pol.n_layers);
    return 0;
}
