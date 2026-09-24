// Parity harness for the Cartan policy: N x obs float32 on stdin -> N x act float32 on stdout.
#include <stdio.h>
#include <stdlib.h>
#include "microduck_infer.h"

#ifndef POLICY_HEADER
#error "define POLICY_HEADER"
#endif
#include POLICY_HEADER

#define CAT_(a, b) a##b
#define CAT(a, b) CAT_(a, b)
#define SYM(s) CAT(POLICY_PREFIX, s)
#define SYMU(s) CAT(POLICY_PREFIX_U, s)

int main(void)
{
    microduck_cartan_t p = {
        .obs_dim = SYMU(_OBS_DIM),
        .act_dim = SYMU(_ACT_DIM),
        .paint = SYMU(_PAINT),
        .n_layers = SYMU(_N_LAYERS),
        .obs_mean = SYM(_obs_mean),
        .obs_std = SYM(_obs_std),
        .in_W = SYM(_in_W),
        .in_b = SYM(_in_b),
        .W = SYM(_W),
        .b = SYM(_b),
        .beta = SYM(_beta),
        .theta = SYM(_theta),
        .head_W = SYM(_head_W),
        .head_b = SYM(_head_b),
        .dilu_alpha = 0.1f,
    };
    float obs[SYMU(_OBS_DIM)];
    float act[SYMU(_ACT_DIM)];
    while (fread(obs, sizeof(float), p.obs_dim, stdin) == p.obs_dim) {
        if (microduck_cartan_forward(&p, obs, act) != 0) {
            fprintf(stderr, "forward failed\n");
            return 2;
        }
        fwrite(act, sizeof(float), p.act_dim, stdout);
    }
    return 0;
}
