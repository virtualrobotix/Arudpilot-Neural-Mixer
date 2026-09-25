// Parity harness: reads N x obs_dim float32 from stdin, writes N x act_dim float32 to stdout.
// Built by tools/parity_check.py against the generated header (POLICY_HEADER / POLICY_PREFIX macros).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nnmixer_infer.h"

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
    nnmixer_policy_t p = {
        .obs_dim = SYMU(_OBS_DIM),
        .act_dim = SYMU(_ACT_DIM),
        .n_layers = SYMU(_N_LAYERS),
        .dims = SYM(_dims),
        .act = SYM(_act),
        .obs_mean = SYM(_obs_mean),
        .obs_std = SYM(_obs_std),
        .W = SYM(_W),
        .b = SYM(_b),
    };
    float obs[SYMU(_OBS_DIM)];
    float act[SYMU(_ACT_DIM)];
    while (fread(obs, sizeof(float), p.obs_dim, stdin) == p.obs_dim) {
        if (nnmixer_forward(&p, obs, act) != 0) {
            fprintf(stderr, "forward failed\n");
            return 2;
        }
        fwrite(act, sizeof(float), p.act_dim, stdout);
    }
    return 0;
}
