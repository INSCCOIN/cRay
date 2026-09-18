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
    s->ns = 8;
    s->sph[0] = (Sph){{0.0, 0.55, 0.2}, 0.55, {0.92, 0.18, 0.16}, 50, 0.40, SHAPE_SPHERE, 0, 0};
    s->sph[1] = (Sph){{-1.4, 0.40, 0.3}, 0.40, {0.18, 0.42, 0.95}, 70, 0.35, SHAPE_BOX, 0.6, 0.25};
    s->sph[2] = (Sph){{1.35, 0.55, 0.15}, 0.32, {0.12, 0.82, 0.38}, 25, 0.12, SHAPE_CYL, 0.4, 0.9};
    s->sph[3] = (Sph){{0.15, 0.22, -1.2}, 0.55, {0.95, 0.88, 0.25}, 80, 0.55, SHAPE_DISK, 0.3, 1.1};
    s->sph[4] = (Sph){{-0.7, 0.28, 1.2}, 0.28, {0.9, 0.9, 0.95}, 90, 0.65, SHAPE_SPHERE, 0, 0};
    s->sph[5] = (Sph){{1.0, 0.30, 1.15}, 0.30, {0.75, 0.25, 0.75}, 40, 0.20, SHAPE_BOX, -0.7, 0.4};
    s->sph[6] = (Sph){{-1.1, 0.50, -0.9}, 0.26, {0.15, 0.15, 0.18}, 20, 0.08, SHAPE_CYL, 1.2, 0.15};
    s->sph[7] = (Sph){{0.6, 0.18, -0.35}, 0.42, {0.2, 0.75, 0.85}, 60, 0.30, SHAPE_DISK, -0.5, 0.7};
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

static Vec world_to_local(const Sph *sp, Vec v)
{
    double cy = cos(sp->yaw), sy = sin(sp->yaw);
    double cp = cos(sp->pitch), spn = sin(sp->pitch);
    Vec a = {v.x * cy - v.z * sy, v.y, v.x * sy + v.z * cy};
    return (Vec){a.x, a.y * cp - a.z * spn, a.y * spn + a.z * cp};
}

static Vec local_to_world(const Sph *sp, Vec v)
{
    double cy = cos(sp->yaw), sy = sin(sp->yaw);
    double cp = cos(sp->pitch), spn = sin(sp->pitch);
    Vec a = {v.x, v.y * cp + v.z * spn, -v.y * spn + v.z * cp};
    return (Vec){a.x * cy + a.z * sy, a.y, -a.x * sy + a.z * cy};
}

const char *shape_name(int s)
{
    static const char *n[] = {"sphere", "box", "cyl", "disk"};
    if (s < 0 || s >= SHAPE_N)
        return "?";
    return n[s];
}

static int hit_box(const Sph *sp, Vec o, Vec d, double *t)
{
    Vec mn = {-sp->r, -sp->r, -sp->r};
    Vec mx = {sp->r, sp->r, sp->r};
    double tmin = -1e9, tmax = 1e9;
    int a;
    double *od = &o.x, *dd = &d.x, *lo = &mn.x, *hi = &mx.x;
    for (a = 0; a < 3; a++) {
        double orig = od[a], dir = dd[a], t1, t2;
        if (fabs(dir) < 1e-12) {
            if (orig < lo[a] || orig > hi[a])
                return 0;
            continue;
        }
        t1 = (lo[a] - orig) / dir;
        t2 = (hi[a] - orig) / dir;
        if (t1 > t2) {
            double sw = t1;
            t1 = t2;
            t2 = sw;
        }
        if (t1 > tmin)
            tmin = t1;
        if (t2 < tmax)
            tmax = t2;
        if (tmax < tmin)
            return 0;
    }
    if (tmin > 1e-4) {
        *t = tmin;
        return 1;
    }
    if (tmax > 1e-4) {
        *t = tmax;
        return 1;
    }
    return 0;
}

static int hit_cyl(const Sph *sp, Vec o, Vec d, double *t)
{
    double r = sp->r, h = sp->r * 1.2;
    double a = d.x * d.x + d.z * d.z;
    double b = 2 * (o.x * d.x + o.z * d.z);
    double c = o.x * o.x + o.z * o.z - r * r;
    double disc, t0, t1, y;
    if (a < 1e-12)
        return 0;
    disc = b * b - 4 * a * c;
    if (disc < 0)
        return 0;
    disc = sqrt(disc);
    t0 = (-b - disc) / (2 * a);
    t1 = (-b + disc) / (2 * a);
    if (t0 > 1e-4) {
        y = o.y + t0 * d.y;
        if (y >= -h && y <= h) {
            *t = t0;
            return 1;
        }
    }
    if (t1 > 1e-4) {
        y = o.y + t1 * d.y;
        if (y >= -h && y <= h) {
            *t = t1;
            return 1;
        }
    }
    return 0;
}

static int hit_disk(const Sph *sp, Vec o, Vec d, double *t)
{
    double tt, x, z;
    if (fabs(d.y) < 1e-8)
        return 0;
    tt = -o.y / d.y;
    if (tt < 1e-4)
        return 0;
    x = o.x + tt * d.x;
    z = o.z + tt * d.z;
    if (x * x + z * z > sp->r * sp->r)
        return 0;
    *t = tt;
    return 1;
}

static int hit_prim(const Sph *sp, Vec o, Vec d, double *t)
{
    Vec ol, dl;
    if (sp->shape == SHAPE_SPHERE)
        return hit_sph(sp, o, d, t);
    ol = world_to_local(sp, vsub(o, sp->c));
    dl = world_to_local(sp, d);
    if (sp->shape == SHAPE_BOX)
        return hit_box(sp, ol, dl, t);
    if (sp->shape == SHAPE_CYL)
        return hit_cyl(sp, ol, dl, t);
    return hit_disk(sp, ol, dl, t);
}

static Vec prim_n(const Sph *sp, Vec p)
{
    Vec l, n;
    if (sp->shape == SHAPE_SPHERE)
        return vnorm(vsub(p, sp->c));
    l = world_to_local(sp, vsub(p, sp->c));
    if (sp->shape == SHAPE_DISK)
        n = (Vec){0, l.y >= 0 ? 1 : -1, 0};
    else if (sp->shape == SHAPE_CYL)
        n = vnorm((Vec){l.x, 0, l.z});
    else {
        double ax = fabs(l.x), ay = fabs(l.y), az = fabs(l.z);
        if (ax >= ay && ax >= az)
            n = (Vec){l.x > 0 ? 1 : -1, 0, 0};
        else if (ay >= ax && ay >= az)
            n = (Vec){0, l.y > 0 ? 1 : -1, 0};
        else
            n = (Vec){0, 0, l.z > 0 ? 1 : -1};
    }
    return vnorm(local_to_world(sp, n));
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
        if (hit_prim(&s->sph[i], o, d, &tt) && tt < best) {
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
        n = prim_n(&s->sph[id], p);
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
