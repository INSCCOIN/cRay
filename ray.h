#ifndef RAY_H
#define RAY_H

typedef struct { double x, y, z; } Vec;

Vec vadd(Vec a, Vec b);
Vec vsub(Vec a, Vec b);
Vec vmul(Vec a, double s);
Vec vhad(Vec a, Vec b);
double vdot(Vec a, Vec b);
Vec vnorm(Vec a);
Vec vref(Vec i, Vec n);

typedef struct {
    Vec c;
    double r;
    Vec col;
    double spec, refl;
} Sph;

typedef struct {
    Vec n, p;
    Vec col;
    double spec, refl;
    int check;
} Plane;

enum { MAX_SPH = 8 };

typedef struct {
    Sph sph[MAX_SPH];
    int ns;
    Plane pl;
    Vec light, lcol;
    Vec eye, look, up;
    double fov;
    int bounce, soft;
    int w, h;
} Scene;

void scene_default(Scene *s);
Vec trace(const Scene *s, Vec o, Vec d, int depth);

#endif
