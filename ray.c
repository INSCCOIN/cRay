#include "ray.h"
#include <math.h>
#include <string.h>

Vec vadd(Vec a, Vec b) { return (Vec){a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec vsub(Vec a, Vec b) { return (Vec){a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec vmul(Vec a, double s) { return (Vec){a.x * s, a.y * s, a.z * s}; }
Vec vhad(Vec a, Vec b) { return (Vec){a.x * b.x, a.y * b.y, a.z * b.z}; }
double vdot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec vnorm(Vec a)
{
    double l = sqrt(vdot(a, a));
    return l > 1e-12 ? vmul(a, 1.0 / l) : a;
}

Vec vref(Vec i, Vec n)
{
    return vsub(i, vmul(n, 2.0 * vdot(i, n)));
}

void scene_default(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->ns = 4;
    s->sph[0] = (Sph){{0, 0.6, 0}, 0.6, {0.9, 0.2, 0.2}, 40, 0.35};
    s->sph[1] = (Sph){{-1.3, 0.45, 0.2}, 0.45, {0.2, 0.4, 0.95}, 60, 0.45};
    s->sph[2] = (Sph){{1.2, 0.35, 0.4}, 0.35, {0.15, 0.8, 0.35}, 20, 0.15};
    s->sph[3] = (Sph){{0.2, 0.25, -1.1}, 0.25, {0.95, 0.85, 0.2}, 80, 0.55};
    s->pl = (Plane){{0, 1, 0}, {0, 0, 0}, {0.75, 0.75, 0.75}, 10, 0.18, 1};
    s->light = (Vec){-2.2, 4.5, 2.5};
    s->lcol = (Vec){1.2, 1.15, 1.05};
    s->eye = (Vec){0, 1.4, 4.2};
    s->look = (Vec){0, 0.4, 0};
    s->up = (Vec){0, 1, 0};
    s->fov = 0.85;
    s->bounce = 3;
    s->soft = 1;
}

static int hit_sph(const Sph *sp, Vec o, Vec d, double *t)
{
    Vec oc = vsub(o, sp->c);
    double a = vdot(d, d), b = 2 * vdot(oc, d), c = vdot(oc, oc) - sp->r * sp->r;
    double disc = b * b - 4 * a * c, s0, s1;
    if (disc < 0)
        return 0;
    disc = sqrt(disc);
    s0 = (-b - disc) / (2 * a);
    s1 = (-b + disc) / (2 * a);
    if (s0 > 1e-4) {
        *t = s0;
        return 1;
    }
    if (s1 > 1e-4) {
        *t = s1;
        return 1;
    }
    return 0;
}

static int hit_pl(const Plane *p, Vec o, Vec d, double *t)
{
    double den = vdot(p->n, d);
    double tt;
    if (fabs(den) < 1e-8)
        return 0;
    tt = vdot(vsub(p->p, o), p->n) / den;
    if (tt < 1e-4)
        return 0;
    *t = tt;
    return 1;
}

static int closest(const Scene *s, Vec o, Vec d, double *t, int *id)
{
    int i, hit = 0;
    double best = 1e9, tt;
    *id = -2;
    for (i = 0; i < s->ns; i++)
        if (hit_sph(&s->sph[i], o, d, &tt) && tt < best) {
            best = tt;
            *id = i;
            hit = 1;
        }
    if (hit_pl(&s->pl, o, d, &tt) && tt < best) {
        best = tt;
        *id = -1;
        hit = 1;
    }
    *t = best;
    return hit;
}

static Vec shade_at(const Scene *s, Vec p, Vec n, Vec col, double spec, double refl, Vec wo, int depth)
{
    Vec ldir, h, c;
    double diff, sh = 1, t;
    int id, k, samples = s->soft ? 4 : 1;
    c = vmul(col, 0.07);
    for (k = 0; k < samples; k++) {
        Vec lp = s->light;
        int sid;
        if (s->soft) {
            lp.x += ((k & 1) ? 0.18 : -0.18);
            lp.z += ((k & 2) ? 0.18 : -0.18);
        }
        ldir = vnorm(vsub(lp, p));
        if (closest(s, vadd(p, vmul(n, 1e-4)), ldir, &t, &sid) && t < sqrt(vdot(vsub(lp, p), vsub(lp, p))))
            sh -= 1.0 / samples;
    }
    if (sh < 0)
        sh = 0;
    ldir = vnorm(vsub(s->light, p));
    diff = vdot(n, ldir);
    if (diff < 0)
        diff = 0;
    c = vadd(c, vhad(col, vmul(s->lcol, diff * sh * 0.85)));
    h = vnorm(vadd(ldir, wo));
    {
        double sp = vdot(n, h);
        if (sp < 0)
            sp = 0;
        sp = pow(sp, spec);
        c = vadd(c, vmul(s->lcol, sp * sh * 0.45));
    }
    if (refl > 0.01 && depth > 0) {
        Vec rd = vref(vmul(wo, -1), n);
        Vec rc = trace(s, vadd(p, vmul(n, 1e-4)), vnorm(rd), depth - 1);
        c = vadd(c, vmul(rc, refl));
    }
    return c;
}

Vec trace(const Scene *s, Vec o, Vec d, int depth)
{
    double t;
    int id;
    Vec p, n, col, wo;
    double spec, refl;
    if (depth < 0 || !closest(s, o, d, &t, &id)) {
        double sky = d.y * 0.5 + 0.5;
        return (Vec){0.12 + 0.35 * sky, 0.16 + 0.40 * sky, 0.28 + 0.45 * sky};
    }
    p = vadd(o, vmul(d, t));
    wo = vnorm(vmul(d, -1));
    if (id >= 0) {
        n = vnorm(vsub(p, s->sph[id].c));
        col = s->sph[id].col;
        spec = s->sph[id].spec;
        refl = s->sph[id].refl;
    } else {
        n = s->pl.n;
        spec = s->pl.spec;
        refl = s->pl.refl;
        col = s->pl.col;
        if (s->pl.check) {
            int cx = (int)floor(p.x + 100) & 1, cz = (int)floor(p.z + 100) & 1;
            if (cx ^ cz)
                col = vmul(col, 0.35);
        }
    }
    return shade_at(s, p, n, col, spec, refl, wo, depth);
}
