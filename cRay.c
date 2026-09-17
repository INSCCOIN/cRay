/* cRay — multithreaded ray tracer + fb GUI. */
#include "ray.h"
#include <fcntl.h>
#include <linux/fb.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#define MAXW 480
#define MAXH 320
#define NTHR 4
#define SAVE "/home/working/cray.ppm"

static int fb = -1;
static unsigned char *map;
static size_t maplen;
static unsigned FW, FH, BPP, LINE;
static struct termios oldt;
static int raw_on;
static uint16_t C_BG, C_TXT, C_DIM, C_HI, C_SEL, C_MENU;

static Scene sc;
static uint8_t img[MAXH][MAXW][3];
static int iw, ih, dirty = 1, rendering, done_rows;
static int menu_i = -1, item_i, run = 1;
static char note[80] = "tab menu  enter render  q";

static uint16_t rgb565(int r, int g, int b)
{
    return (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

static void px(int x, int y, uint16_t c)
{
    unsigned char *p;
    if ((unsigned)x >= FW || (unsigned)y >= FH)
        return;
    p = map + (size_t)y * LINE + (size_t)x * (BPP / 8);
    if (BPP == 16)
        ((uint16_t *)p)[0] = c;
    else if (BPP == 32) {
        p[0] = (unsigned char)((c & 0x1f) << 3);
        p[1] = (unsigned char)(((c >> 5) & 0x3f) << 2);
        p[2] = (unsigned char)(((c >> 11) & 0x1f) << 3);
        p[3] = 0;
    }
}

static void fill_rect(int x, int y, int w, int h, uint16_t c)
{
    int i, j;
    for (j = 0; j < h; j++)
        for (i = 0; i < w; i++)
            px(x + i, y + j, c);
}

static const unsigned char FONT[96][5] = {
    {0,0,0,0,0},{0,0,0x5f,0,0},{0,7,0,7,0},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,8,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},
    {0,5,3,0,0},{0,0x1c,0x22,0x41,0},{0,0x41,0x22,0x1c,0},{0x14,8,0x3e,8,0x14},
    {8,8,0x3e,8,8},{0,0x50,0x30,0,0},{8,8,8,8,8},{0,0x60,0x60,0,0},
    {0x20,0x10,8,4,2},{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},
    {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e},{0,0x36,0x36,0,0},
    {0,0x56,0x36,0,0},{8,0x14,0x22,0x41,0},{0x14,0x14,0x14,0x14,0x14},
    {0,0x41,0x22,0x14,8},{2,1,0x51,9,6},{0x32,0x49,0x79,0x41,0x3e},
    {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},
    {0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},{0,0x41,0x7f,0x41,0},
    {0x20,0x40,0x41,0x3f,1},{0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
    {0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,9,9,9,6},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},{0x3f,0x40,0x40,0x40,0x3f},
    {0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43},{0,0x7f,0x41,0x41,0},
    {2,4,8,0x10,0x20},{0,0x41,0x41,0x7f,0},{4,2,1,2,4},{0x40,0x40,0x40,0x40,0x40},
    {0,1,2,4,0},{0x20,0x54,0x54,0x54,0x78},{0x7f,0x48,0x44,0x44,0x38},
    {0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7f},{0x38,0x54,0x54,0x54,0x18},
    {8,0x7e,9,1,2},{0x0c,0x52,0x52,0x52,0x3e},{0x7f,8,4,4,0x78},
    {0,0x44,0x7d,0x40,0},{0x20,0x40,0x44,0x3d,0},{0x7f,0x10,0x28,0x44,0},
    {0,0x41,0x7f,0x40,0},{0x7c,4,0x18,4,0x78},{0x7c,8,4,4,0x78},
    {0x38,0x44,0x44,0x44,0x38},{0x7c,0x14,0x14,0x14,8},{8,0x14,0x14,0x18,0x7c},
    {0x7c,8,4,4,8},{0x48,0x54,0x54,0x54,0x20},{4,0x3f,0x44,0x40,0x20},
    {0x3c,0x40,0x40,0x20,0x7c},{0x1c,0x20,0x40,0x20,0x1c},{0x3c,0x40,0x30,0x40,0x3c},
    {0x44,0x28,0x10,0x28,0x44},{0x0c,0x50,0x50,0x50,0x3c},{0x44,0x64,0x54,0x4c,0x44},
};

static void text(int x, int y, const char *s, uint16_t c)
{
    while (*s) {
        unsigned char ch = (unsigned char)*s++;
        int gx, gy;
        unsigned char col;
        if (ch < 32 || ch > 126)
            ch = '?';
        for (gx = 0; gx < 5; gx++) {
            col = FONT[ch - 32][gx];
            for (gy = 0; gy < 7; gy++)
                if (col & (1 << gy))
                    px(x + gx, y + gy, c);
        }
        x += 6;
    }
}

static int fb_open(void)
{
    struct fb_var_screeninfo v;
    struct fb_fix_screeninfo f;
    fb = open("/dev/fb0", O_RDWR);
    if (fb < 0)
        return -1;
    ioctl(fb, FBIOGET_VSCREENINFO, &v);
    ioctl(fb, FBIOGET_FSCREENINFO, &f);
    FW = v.xres;
    FH = v.yres;
    BPP = v.bits_per_pixel;
    LINE = f.line_length;
    maplen = f.smem_len ? f.smem_len : (size_t)LINE * FH;
    map = mmap(NULL, maplen, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    return map == MAP_FAILED ? -1 : 0;
}

typedef struct {
    int y0, y1;
} Job;

static Vec cam_fwd, cam_r, cam_u;

static void cam_basis(void)
{
    cam_fwd = vnorm(vsub(sc.look, sc.eye));
    cam_r = vnorm((Vec){cam_fwd.z, 0, -cam_fwd.x});
    if (fabs(cam_r.x) + fabs(cam_r.z) < 1e-6)
        cam_r = (Vec){1, 0, 0};
    cam_u = vnorm((Vec){
        cam_r.y * cam_fwd.z - cam_r.z * cam_fwd.y,
        cam_r.z * cam_fwd.x - cam_r.x * cam_fwd.z,
        cam_r.x * cam_fwd.y - cam_r.y * cam_fwd.x});
}

static void *worker(void *arg)
{
    Job *j = arg;
    int y, x;
    double aspect = (double)iw / ih;
    for (y = j->y0; y < j->y1; y++) {
        for (x = 0; x < iw; x++) {
            double u = (2.0 * (x + 0.5) / iw - 1.0) * tan(sc.fov * 0.5) * aspect;
            double v = (1.0 - 2.0 * (y + 0.5) / ih) * tan(sc.fov * 0.5);
            Vec dir = vnorm(vadd(cam_fwd, vadd(vmul(cam_r, u), vmul(cam_u, v))));
            Vec c = trace(&sc, sc.eye, dir, sc.bounce);
            int r = (int)(c.x * 255), g = (int)(c.y * 255), b = (int)(c.z * 255);
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            if (r < 0) r = 0;
            if (g < 0) g = 0;
            if (b < 0) b = 0;
            img[y][x][0] = (uint8_t)r;
            img[y][x][1] = (uint8_t)g;
            img[y][x][2] = (uint8_t)b;
        }
        __sync_add_and_fetch(&done_rows, 1);
    }
    return NULL;
}

static void render_pass(int w, int h)
{
    pthread_t th[NTHR];
    Job job[NTHR];
    int i, sl;
    iw = w;
    ih = h;
    if (iw > MAXW)
        iw = MAXW;
    if (ih > MAXH)
        ih = MAXH;
    cam_basis();
    done_rows = 0;
    rendering = 1;
    sl = ih / NTHR;
    for (i = 0; i < NTHR; i++) {
        job[i].y0 = i * sl;
        job[i].y1 = (i == NTHR - 1) ? ih : (i + 1) * sl;
        pthread_create(&th[i], NULL, worker, &job[i]);
    }
    for (i = 0; i < NTHR; i++)
        pthread_join(th[i], NULL);
    rendering = 0;
    dirty = 1;
}

static void save_ppm(void)
{
    FILE *f = fopen(SAVE, "wb");
    int y, x;
    if (!f)
        return;
    fprintf(f, "P6\n%d %d\n255\n", iw, ih);
    for (y = 0; y < ih; y++)
        for (x = 0; x < iw; x++)
            fwrite(img[y][x], 1, 3, f);
    fclose(f);
}

static void blit(void)
{
    int x, y, ox, oy;
    if (!iw || !ih)
        return;
    ox = ((int)FW - iw) / 2;
    oy = 16 + ((int)FH - 30 - ih) / 2;
    if (oy < 16)
        oy = 16;
    for (y = 0; y < ih; y++)
        for (x = 0; x < iw; x++)
            px(ox + x, oy + y, rgb565(img[y][x][0], img[y][x][1], img[y][x][2]));
}

static const char *MENUS[] = {"Render", "Qual", "Scene", "File"};
#define NM 4

static int nitems(void)
{
    if (menu_i == 0)
        return 3;
    if (menu_i == 1)
        return 4;
    if (menu_i == 2)
        return 3;
    if (menu_i == 3)
        return 2;
    return 0;
}

static void item_txt(int i, char *o, size_t n)
{
    if (menu_i == 0) {
        const char *s[] = {"Preview 160x100", "Half 240x160", "Full 480x288"};
        snprintf(o, n, "%s", s[i]);
    } else if (menu_i == 1) {
        if (i == 0)
            snprintf(o, n, "bounce %d", sc.bounce);
        else if (i == 1)
            snprintf(o, n, "soft %s", sc.soft ? "on" : "off");
        else if (i == 2)
            snprintf(o, n, "fov %.2f", sc.fov);
        else
            snprintf(o, n, "4 threads");
    } else if (menu_i == 2) {
        const char *s[] = {"default scene", "eye +", "eye -"};
        snprintf(o, n, "%s", s[i]);
    } else {
        const char *s[] = {"save PPM", "quit"};
        snprintf(o, n, "%s", s[i]);
    }
}

static void do_item(void)
{
    if (menu_i == 0) {
        if (item_i == 0)
            render_pass(160, 100);
        else if (item_i == 1)
            render_pass(240, 160);
        else
            render_pass((int)FW, (int)FH > 30 ? (int)FH - 30 : 200);
        snprintf(note, sizeof note, "done %dx%d", iw, ih);
    } else if (menu_i == 1) {
        if (item_i == 0)
            sc.bounce = sc.bounce >= 5 ? 1 : sc.bounce + 1;
        else if (item_i == 1)
            sc.soft ^= 1;
        else if (item_i == 2)
            sc.fov = sc.fov > 1.1 ? 0.55 : sc.fov + 0.1;
        snprintf(note, sizeof note, "qual b%d s%d", sc.bounce, sc.soft);
    } else if (menu_i == 2) {
        if (item_i == 0)
            scene_default(&sc);
        else if (item_i == 1)
            sc.eye.z -= 0.35;
        else
            sc.eye.z += 0.35;
        snprintf(note, sizeof note, "scene");
    } else if (menu_i == 3) {
        if (item_i == 0) {
            save_ppm();
            snprintf(note, sizeof note, "wrote %s", SAVE);
        } else
            run = 0;
    }
}

static void draw_ui(void)
{
    int m, i, mx, n;
    char line[48];
    fill_rect(0, 0, (int)FW, 16, C_DIM);
    fill_rect(0, (int)FH - 14, (int)FW, 14, C_DIM);
    for (m = 0; m < NM; m++)
        text(4 + m * 64, 5, MENUS[m], menu_i == m ? C_SEL : C_TXT);
    text(4, (int)FH - 10, note, C_TXT);
    if (menu_i < 0)
        return;
    n = nitems();
    mx = 4 + menu_i * 64;
    fill_rect(mx, 16, 130, 8 + n * 10, C_MENU);
    for (i = 0; i < n; i++) {
        item_txt(i, line, sizeof line);
        text(mx + 4, 20 + i * 10, line, i == item_i ? C_SEL : C_TXT);
    }
}

static void frame(void)
{
    if (dirty) {
        fill_rect(0, 16, (int)FW, (int)FH - 30, C_BG);
        blit();
        dirty = 0;
    }
    draw_ui();
}

static void raw(int on)
{
    struct termios t;
    if (on) {
        tcgetattr(0, &oldt);
        t = oldt;
        t.c_lflag &= ~(ICANON | ECHO);
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(0, TCSANOW, &t);
        raw_on = 1;
    } else if (raw_on)
        tcsetattr(0, TCSANOW, &oldt);
}

int main(void)
{
    if (fb_open() < 0) {
        fprintf(stderr, "cRay: /dev/fb0\n");
        return 1;
    }
    C_BG = rgb565(10, 12, 16);
    C_TXT = rgb565(210, 215, 220);
    C_DIM = rgb565(18, 22, 28);
    C_HI = rgb565(80, 220, 140);
    C_SEL = rgb565(255, 220, 80);
    C_MENU = rgb565(16, 22, 32);
    scene_default(&sc);
    raw(1);
    render_pass(160, 100);
    snprintf(note, sizeof note, "preview %dx%d  tab menu", iw, ih);
    dirty = 1;
    frame();
    while (run) {
        unsigned char ch = 0;
        fd_set rf;
        struct timeval tv = {0, 50000};
        FD_ZERO(&rf);
        FD_SET(0, &rf);
        if (select(1, &rf, NULL, NULL, &tv) > 0)
            if (read(0, &ch, 1) != 1)
                ch = 0;
        if (!ch)
            continue;
        if (ch == '\t') {
            menu_i = (menu_i + 1) % NM;
            item_i = 0;
            frame();
            continue;
        }
        if (ch == 0x1b) {
            unsigned char seq[8] = {0};
            struct timeval t2 = {0, 80000};
            FD_ZERO(&rf);
            FD_SET(0, &rf);
            if (select(1, &rf, NULL, NULL, &t2) > 0)
                read(0, seq, 6);
            if (seq[0] == 0)
                menu_i = -1;
            else if (menu_i >= 0 && seq[0] == '[' && seq[1] == 'A' && item_i)
                item_i--;
            else if (menu_i >= 0 && seq[0] == '[' && seq[1] == 'B' && item_i + 1 < nitems())
                item_i++;
            else if (menu_i >= 0 && seq[0] == '[' && seq[1] == 'C') {
                menu_i = (menu_i + 1) % NM;
                item_i = 0;
            } else if (menu_i >= 0 && seq[0] == '[' && seq[1] == 'D') {
                menu_i = menu_i <= 0 ? NM - 1 : menu_i - 1;
                item_i = 0;
            } else
                menu_i = -1;
            frame();
            continue;
        }
        if (ch == 'q')
            run = 0;
        else if (ch == 10 || ch == 13) {
            if (menu_i < 0)
                menu_i = 0;
            else
                do_item();
            dirty = 1;
            frame();
        } else if (ch == 'r') {
            render_pass(160, 100);
            snprintf(note, sizeof note, "preview");
            frame();
        }
    }
    raw(0);
    munmap(map, maplen);
    close(fb);
    return 0;
}
