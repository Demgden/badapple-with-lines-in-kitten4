/*
 * ============================================================
 *  视频 -> 线段数据  (bad apple 影绘 / 黑白剪影专用)
 * ============================================================
 *
 *  原理：
 *    1. 用 ffmpeg 把视频解码成 1280x720 的灰度原始帧，通过管道读进内存；
 *    2. 每帧做 3x3 高斯模糊（压掉胶片噪点），再按阈值二值化；
 *    3. 用"等值线追踪"(Marching Squares) 找出所有黑/白交界处的轮廓，
 *       每一条轮廓就是一条首尾相连的折线（= 你说的"相连线段"）；
 *    4. Douglas-Peucker 抽稀，把几万个点压到几百个点，形状基本不变；
 *    5. 按要求的 4 个文件格式输出。
 *
 *  输出文件（都在程序当前目录；坐标 0~1279 / 0~719，原点左上角，y 向下）：
 *    x.txt        一行 = 一条折线；行内是该折线所有顶点的 x，用逗号隔开
 *                （每条折线首尾是同一个点，即图形自动闭合）
 *    y.txt        一行 = 一条折线；行内是该折线所有顶点的 y，用逗号隔开
 *                 （x.txt 与 y.txt 行数相同，第 k 行是同一条折线）
 *    帧索引.txt    每行 = 该帧第一条折线在 x.txt 中的行号（从 1 开始）
 *                 末行多一个哨兵 = 折线总行数 + 1（最后一条折线的下一行）
 *
 *  ★ 每条折线代表"一块白色区域"，内部保证只有白色，不会包住黑色。
 *    所以直接"把每条折线内部填白"就能得到正确的画面（黑色 = 底图不画）。
 *    如果一块白色区域中间夹着黑色（比如白背景里有个黑苹果），程序会用一条
 *    零宽的缝把黑色区域从这条折线里减掉：去一趟、绕黑洞一圈、原路返回。
 *    缝本身在画面上是一条细线，不影响填充结果（奇偶规则、非零规则都能挖掉）。
 *
 *  读取方法：
 *    第 f 帧： i = 帧索引[f]， j = 帧索引[f+1]， 本帧折线条数 n = j - i
 *    第 k 条折线（k = i .. j-1，1 基）：看 x.txt / y.txt 的第 k 行，
 *        拿逗号切开，第 m 个 x 和第 m 个 y 组成一个顶点，按顺序连起来、内部填白
 *    某帧的 n = 0 表示这帧全黑（什么都不用画）；
 *    整帧全白时会给一个覆盖 0,0 ~ 1279,719 的矩形折线。
 *
 *  用法：
 *    直接双击运行 -> 会问你读取帧率，回车用默认值
 *    命令行：  lines.exe 24            （帧率 24）
 *              lines.exe 12 1.5        （帧率 12，抽稀容差 1.5）
 *              lines.exe 24 1.2 300    （帧率 24，容差 1.2，只处理前 300 帧）
 *
 *  编译：Dev-C++ 5.11 直接编译即可（不需要额外库，ffmpeg 需在 PATH 里）
 * ============================================================
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <map>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

/* ==================== 配置（一般只改前两项） ==================== */

// 视频路径（相对程序所在目录，也可以写绝对路径）
#ifndef VIDEO_PATH
#define VIDEO_PATH "../badapple影绘.mp4"
#endif

// ffmpeg 程序（已加进 PATH 就写 "ffmpeg"，否则写完整路径）
#ifndef FFMPEG_EXE
#define FFMPEG_EXE "ffmpeg"
#endif

// 默认读取帧率（运行时可以改；视频原速 24fps）
#ifndef FPS
#define FPS 24
#endif

// 只处理前 N 帧（0 = 全部），运行时也能改
#ifndef MAX_FRAMES
#define MAX_FRAMES 0
#endif

// 输出分辨率（和视频一致，1:1）
static const int OUT_W = 1280;
static const int OUT_H = 720;

// 二值化阈值 0~255：小于它算"黑"（影绘主体）
static const int ISO = 80;

// 高斯模糊次数，0 = 不模糊；噪点多可以试 2
static const int BLUR_TIMES = 2;

// 折线抽稀容差（像素），越大点越少越生硬，1.0~1.5 比较合适
static const double SIMPLIFY_EPS = 2.5;

// 过滤小噪点：折线总长 / 包围盒面积小于下面值就丢掉
static const double MIN_POLY_LEN  = 12.0;
static const double MIN_POLY_AREA = 24.0;

// 输出文件名
static const char *OUT_X     = "x.txt";
static const char *OUT_Y     = "y.txt";
static const char *OUT_FRAME = "帧索引.txt";
// 输出换行符：Windows 记事本等工具需要 \r\n，只给 \n 的话整份文件会挤成一行
static const bool OUT_CRLF = true;

/* ==================== 以下不用改 ==================== */

#ifdef _WIN32
// 把源码里的字符串转成宽字符：先按 UTF-8 试，不合法就按系统 ANSI(GBK)，
// 所以源文件存成 UTF-8 或 ANSI 都不会把中文路径搞坏。
static wstring toWide(const char *s)
{
    if (!s) return wstring();
    UINT cp = CP_UTF8;
    int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s, -1, NULL, 0);
    if (n <= 0) {
        cp = CP_ACP;
        n = MultiByteToWideChar(CP_ACP, 0, s, -1, NULL, 0);
        if (n <= 0) return wstring();
    }
    vector<wchar_t> buf(n);
    MultiByteToWideChar(cp, (cp == CP_UTF8 ? MB_ERR_INVALID_CHARS : 0),
                        s, -1, &buf[0], n);
    return wstring(&buf[0]);
}

// 转绝对路径。这一步是必需的：cmd.exe 不认 "../ffmpeg.exe" 这种正斜杠相对路径，
// 它会把它拆成 命令 "../" + 参数 "/ffmpeg.exe"，于是报".. 不是内部或外部命令"。
static string absPath(const char *utf8path)
{
    wstring w = toWide(utf8path);
    wchar_t full[MAX_PATH * 4];
    DWORD n = GetFullPathNameW(w.c_str(), MAX_PATH * 4, full, NULL);
    if (n == 0 || n >= MAX_PATH * 4) return string(utf8path);
    int len = WideCharToMultiByte(CP_UTF8, 0, full, -1, NULL, 0, NULL, NULL);
    if (len <= 0) return string(utf8path);
    string s(len - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, full, -1, &s[0], len, NULL, NULL);
    return s;
}

// 给 cmd 用的路径：先转绝对路径，再优先取 8.3 短路径（纯 ASCII、无空格，
// 绝对不会因编码或空格出错），取不到就退回 GBK/ANSI。
static string cmdPath(const char *utf8path)
{
    string ab = absPath(utf8path);
    wstring w = toWide(ab.c_str());
    DWORD n = GetShortPathNameW(w.c_str(), NULL, 0);
    if (n > 0) {
        vector<wchar_t> b(n);
        DWORD m = GetShortPathNameW(w.c_str(), &b[0], n);
        if (m > 0 && m < n) {
            bool ascii = true;
            for (DWORD i = 0; i < m; ++i)
                if ((unsigned)b[i] > 127) { ascii = false; break; }
            if (ascii) {
                string s;
                for (DWORD i = 0; i < m; ++i) s += (char)b[i];
                return s;
            }
        }
    }
    int len = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, NULL, 0, NULL, NULL);
    if (len > 0) {
        string s(len - 1, '\0');
        WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, &s[0], len, NULL, NULL);
        return s;
    }
    return ab;
}

static bool fileExists(const char *utf8path)
{
    wstring w = toWide(utf8path);
    return GetFileAttributesW(w.c_str()) != INVALID_FILE_ATTRIBUTES;
}

// 找 ffmpeg：1) 顶部写的是带目录的路径就用它 2) 在 PATH/系统目录里搜
//            3) 找不到就用视频同目录的 ffmpeg.exe。一律返回绝对路径。
static string ffmpegPath()
{
    string f = FFMPEG_EXE;

    // 1) 顶部填的是带目录的路径
    if (f.find('/') != string::npos || f.find('\\') != string::npos) {
        if (fileExists(f.c_str())) return absPath(f.c_str());
    }

    // 2) PATH / 系统目录（SearchPathW 命中时返回的就是绝对路径）
    {
        wstring wf = toWide(f.c_str());
        wchar_t buf[MAX_PATH * 4];
        DWORD n = SearchPathW(NULL, wf.c_str(), L".exe", MAX_PATH * 4, buf, NULL);
        if (n > 0 && n < MAX_PATH * 4) {
            int len = WideCharToMultiByte(CP_UTF8, 0, buf, -1, NULL, 0, NULL, NULL);
            if (len > 0) {
                string s(len - 1, '\0');
                WideCharToMultiByte(CP_UTF8, 0, buf, -1, &s[0], len, NULL, NULL);
                return s;
            }
        }
    }

    // 3) 视频同目录
    string v = absPath(VIDEO_PATH);
    size_t p = v.find_last_of("/\\");
    string cand = (p == string::npos) ? string("ffmpeg.exe")
                                      : (v.substr(0, p + 1) + "ffmpeg.exe");
    if (fileExists(cand.c_str())) return cand;
    return f;
}
#endif

/* ---------------- 输出缓冲（攒够 1MB 再写盘） ---------------- */

struct OutFile {
    FILE *f;
    string buf;
    OutFile() : f(NULL) { buf.reserve(1 << 20); }

    bool open(const char *name)
    {
#ifdef _WIN32
        wstring w = toWide(name);
        f = _wfopen(w.c_str(), L"wb");
#else
        f = fopen(name, "wb");
#endif
        return f != NULL;
    }
    void flush()
    {
        if (f && !buf.empty()) { fwrite(buf.data(), 1, buf.size(), f); buf.clear(); }
    }
    void putInt(long long v)
    {
        char t[24];
        int n = 0;
        if (v == 0) t[n++] = '0';
        else {
            bool neg = v < 0;
            if (neg) v = -v;
            while (v > 0) { t[n++] = (char)('0' + (int)(v % 10)); v /= 10; }
            if (neg) t[n++] = '-';
        }
        for (int i = n - 1; i >= 0; --i) buf.push_back(t[i]);
        if (buf.size() >= (1u << 20) - 32) flush();
    }
    void comma()
    {
        buf.push_back(',');
        if (buf.size() >= (1u << 20) - 32) flush();
    }
    void nl()
    {
        if (OUT_CRLF) buf.push_back('\r');
        buf.push_back('\n');
        if (buf.size() >= (1u << 20) - 32) flush();
    }
    void close()
    {
        if (f) { flush(); fclose(f); f = NULL; }
    }
};

/* ---------------- 等值线追踪表 (Marching Squares) ----------------
 * 格子四角： v0=(x,y)  v1=(x+1,y)  v2=(x+1,y+1)  v3=(x,y+1)
 * 位：      1=v0黑 2=v1黑 4=v2黑 8=v3黑
 * 边编号：  0=上 1=右 2=下 3=左
 * 每格 {第一段起点边, 第一段终点边, 第二段起点边, 第二段终点边}，-1 表示没有
 */
static const int MS_TAB[16][4] = {
    { -1, -1, -1, -1 },   // 0  全白
    {  0,  3, -1, -1 },   // 1
    {  0,  1, -1, -1 },   // 2
    {  3,  1, -1, -1 },   // 3
    {  1,  2, -1, -1 },   // 4
    {  0,  1,  2,  3 },   // 5  鞍点
    {  0,  2, -1, -1 },   // 6
    {  3,  2, -1, -1 },   // 7
    {  2,  3, -1, -1 },   // 8
    {  0,  2, -1, -1 },   // 9
    {  0,  3,  1,  2 },   // 10 鞍点
    {  1,  2, -1, -1 },   // 11
    {  3,  1, -1, -1 },   // 12
    {  0,  1, -1, -1 },   // 13
    {  0,  3, -1, -1 },   // 14
    { -1, -1, -1, -1 }    // 15 全黑
};

struct Seg {
    float x0, y0, x1, y1;
    long long k0, k1;
};

struct PolyRec {
    vector<float> xs, ys;
};

// 取格子上第 e 条边上的交点。
// 键只用 (边朝向, 行, 列)：同一根网格边被两个格子共享，算出的 t 完全一样，
// 所以键必然对得上，不需要把 t 也编进去。
static inline void edgePoint(int e, int x, int y,
                             int v0, int v1, int v2, int v3, float iso,
                             float &px, float &py, long long &key)
{
    float t;
    switch (e) {
    case 0:  // 上边 (x,y)-(x+1,y)
        t = (iso - v0) / (float)(v1 - v0);
        px = (float)x + t; py = (float)y;
        key = (0LL << 40) | ((long long)y << 20) | (long long)x;
        break;
    case 1:  // 右边 (x+1,y)-(x+1,y+1)
        t = (iso - v1) / (float)(v2 - v1);
        px = (float)x + 1.0f; py = (float)y + t;
        key = (1LL << 40) | ((long long)y << 20) | (long long)(x + 1);
        break;
    case 2:  // 下边 (x,y+1)-(x+1,y+1)
        t = (iso - v3) / (float)(v2 - v3);
        px = (float)x + t; py = (float)y + 1.0f;
        key = (0LL << 40) | ((long long)(y + 1) << 20) | (long long)x;
        break;
    default: // 3 左边 (x,y)-(x,y+1)
        t = (iso - v0) / (float)(v3 - v0);
        px = (float)x; py = (float)y + t;
        key = (1LL << 40) | ((long long)y << 20) | (long long)x;
        break;
    }
}

/* ---------------- Douglas-Peucker 抽稀 ---------------- */

static double distToSeg(double px, double py,
                        double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1, dy = y2 - y1;
    double len2 = dx * dx + dy * dy;
    if (len2 <= 1e-12) {
        double ax = px - x1, ay = py - y1;
        return sqrt(ax * ax + ay * ay);
    }
    double t = ((px - x1) * dx + (py - y1) * dy) / len2;
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    double qx = x1 + t * dx, qy = y1 + t * dy;
    double ax = px - qx, ay = py - qy;
    return sqrt(ax * ax + ay * ay);
}

static void dpSimplify(const vector<float> &xs, const vector<float> &ys,
                       double eps, vector<char> &keep)
{
    int n = (int)xs.size();
    keep.assign(n, 0);
    if (n == 0) return;
    keep[0] = 1;
    keep[n - 1] = 1;

    vector<pair<int, int> > st;
    st.push_back(make_pair(0, n - 1));
    while (!st.empty()) {
        int a = st.back().first, b = st.back().second;
        st.pop_back();
        if (b - a < 2) continue;
        double best = -1.0;
        int bi = -1;
        for (int i = a + 1; i < b; ++i) {
            double d = distToSeg(xs[i], ys[i], xs[a], ys[a], xs[b], ys[b]);
            if (d > best) { best = d; bi = i; }
        }
        if (bi > 0 && best > eps) {
            keep[bi] = 1;
            st.push_back(make_pair(a, bi));
            st.push_back(make_pair(bi, b));
        }
    }
}

/* ---------------- 一帧灰度图 -> 若干条折线 ---------------- */

static vector<Seg>   g_segs;
static vector<char>  g_used;
static map<long long, vector<int> > g_ends;   // 键 -> 线段端点编号(seg*2+端)

static void traceContours(const unsigned char *img, vector<PolyRec> &out)
{
    const int W = OUT_W, H = OUT_H;
    const float iso = (float)ISO;

    g_segs.clear();
    g_segs.reserve(4096);
    g_ends.clear();

    for (int y = 0; y < H - 1; ++y) {
        const unsigned char *r0 = img + (size_t)y * W;
        const unsigned char *r1 = r0 + W;
        for (int x = 0; x < W - 1; ++x) {
            int v0 = r0[x], v1 = r0[x + 1], v2 = r1[x + 1], v3 = r1[x];
            int c = 0;
            if (v0 < ISO) c |= 1;
            if (v1 < ISO) c |= 2;
            if (v2 < ISO) c |= 4;
            if (v3 < ISO) c |= 8;
            if (c == 0 || c == 15) continue;

            const int *tb = MS_TAB[c];
            for (int s = 0; s < 2; ++s) {
                int ea = tb[s * 2], eb = tb[s * 2 + 1];
                if (ea < 0) break;
                Seg sg;
                edgePoint(ea, x, y, v0, v1, v2, v3, iso, sg.x0, sg.y0, sg.k0);
                edgePoint(eb, x, y, v0, v1, v2, v3, iso, sg.x1, sg.y1, sg.k1);
                int idx = (int)g_segs.size();
                g_segs.push_back(sg);
                g_ends[sg.k0].push_back(idx * 2 + 0);
                g_ends[sg.k1].push_back(idx * 2 + 1);
            }
        }
    }

    int nseg = (int)g_segs.size();
    if (nseg == 0) return;
    g_used.assign(nseg, 0);

    vector<float> fxs, fys;   // 正向链
    vector<float> bxs, bys;   // 反向链

    for (int i = 0; i < nseg; ++i) {
        if (g_used[i]) continue;
        g_used[i] = 1;

        const Seg &s0 = g_segs[i];
        fxs.clear(); fys.clear();
        fxs.push_back(s0.x0); fys.push_back(s0.y0);
        fxs.push_back(s0.x1); fys.push_back(s0.y1);

        // 从尾端继续往前接
        long long k = s0.k1;
        while (true) {
            map<long long, vector<int> >::iterator it = g_ends.find(k);
            if (it == g_ends.end()) break;
            int nxt = -1;
            for (size_t j = 0; j < it->second.size(); ++j) {
                int occ = it->second[j];
                if (!g_used[occ >> 1]) { nxt = occ; break; }
            }
            if (nxt < 0) break;
            int sid = nxt >> 1;
            g_used[sid] = 1;
            const Seg &sg = g_segs[sid];
            if ((nxt & 1) == 0) { fxs.push_back(sg.x0); fys.push_back(sg.y0); k = sg.k1; }
            else                { fxs.push_back(sg.x1); fys.push_back(sg.y1); k = sg.k0; }
            if (fxs.size() > 400000u) break;
        }

        // 从头端往后接（这一段要倒着拼到前面）
        k = s0.k0;
        bxs.clear(); bys.clear();
        while (true) {
            map<long long, vector<int> >::iterator it = g_ends.find(k);
            if (it == g_ends.end()) break;
            int nxt = -1;
            for (size_t j = 0; j < it->second.size(); ++j) {
                int occ = it->second[j];
                if (!g_used[occ >> 1]) { nxt = occ; break; }
            }
            if (nxt < 0) break;
            int sid = nxt >> 1;
            g_used[sid] = 1;
            const Seg &sg = g_segs[sid];
            if ((nxt & 1) == 0) { bxs.push_back(sg.x0); bys.push_back(sg.y0); k = sg.k1; }
            else                { bxs.push_back(sg.x1); bys.push_back(sg.y1); k = sg.k0; }
            if (bxs.size() > 400000u) break;
        }

        PolyRec pl;
        pl.xs.reserve(bxs.size() + fxs.size());
        pl.ys.reserve(bxs.size() + fxs.size());
        for (int j = (int)bxs.size() - 1; j >= 0; --j) {
            pl.xs.push_back(bxs[j]);
            pl.ys.push_back(bys[j]);
        }
        for (size_t j = 0; j < fxs.size(); ++j) {
            pl.xs.push_back(fxs[j]);
            pl.ys.push_back(fys[j]);
        }
        out.push_back(pl);
    }
}

/* ---------------- 把轮廓整理成"纯色区域" ----------------
 * 目标：每条折线围出的面积里只能有一种颜色。
 *   用户的播放器把折线内部一律填白、黑色靠底图，所以折线内部绝不能再包住黑色，
 *   否则一填白就把黑色的形状盖掉了（13~14 秒的苹果就是这个情况）。
 *
 * 做法：
 *   1) 判断每条轮廓内部是白还是黑（在轮廓内侧取几个点采样，投票）；
 *   2) 用"谁包含谁"建父子关系，只输出"内部是白"的轮廓；
 *   3) 如果它里面夹着黑色区域，就用一条零宽的缝（去一趟、原路返回）
 *      把黑色区域从这条折线里减掉。填充时无论用奇偶规则还是非零规则，
 *      被减掉的黑色区域都不会被涂白。缝本身会显示成一条细线，无所谓。
 *
 *   这样每条折线的内部就只剩白色，随便怎么填都不会互相打架。
 */

struct Region {
    vector<int> xs, ys;      // 已闭合（首尾同点）
    double area;             // 有向面积
    int color;               // 1 = 内部是白, 0 = 内部是黑
    int parent;              // 直接包含它的那条轮廓，-1 表示没有被包含
};

// 有向面积（屏幕坐标，y 向下）
static double polyArea(const vector<int> &xs, const vector<int> &ys)
{
    double s = 0.0;
    int n = (int)xs.size();
    for (int i = 0; i + 1 < n; ++i)
        s += (double)xs[i] * ys[i + 1] - (double)xs[i + 1] * ys[i];
    return s * 0.5;
}

// 点是否在多边形内部（射线法）
static bool pointInPoly(double px, double py, const vector<int> &xs, const vector<int> &ys)
{
    bool in = false;
    int n = (int)xs.size();
    for (int i = 0; i + 1 < n; ++i) {
        double x1 = xs[i], y1 = ys[i], x2 = xs[i + 1], y2 = ys[i + 1];
        if ((y1 > py) != (y2 > py)) {
            double xc = x1 + (py - y1) * (x2 - x1) / (y2 - y1);
            if (px < xc) in = !in;
        }
    }
    return in;
}

// 浮点版：点是否在多边形内部
static bool pointInPolyF(double px, double py, const vector<float> &xs, const vector<float> &ys)
{
    bool in = false;
    int n = (int)xs.size();
    for (int i = 0; i + 1 < n; ++i) {
        double x1 = xs[i], y1 = ys[i], x2 = xs[i + 1], y2 = ys[i + 1];
        if ((y1 > py) != (y2 > py)) {
            double xc = x1 + (py - y1) * (x2 - x1) / (y2 - y1);
            if (px < xc) in = !in;
        }
    }
    return in;
}

// 判断轮廓内部是白(1)还是黑(0)。
// 一定要用"抽稀之前"的原始轮廓：eps=10 会把细长的轮廓压成几根线，
// 拿压扁后的多边形去采样会判错（细白线会被当成黑、细黑线会被当成白）。
static int innerColorF(const unsigned char *img, const vector<float> &xs, const vector<float> &ys)
{
    int n = (int)xs.size();
    int white = 0, black = 0;
    for (int i = 0; i + 1 < n; ++i) {
        double dx = xs[i + 1] - xs[i], dy = ys[i + 1] - ys[i];
        double len = sqrt(dx * dx + dy * dy);
        if (len < 1e-9) continue;
        double nx = -dy / len, ny = dx / len;
        double mx = (xs[i] + xs[i + 1]) * 0.5, my = (ys[i] + ys[i + 1]) * 0.5;
        for (int side = 0; side < 2; ++side) {
            double sgn = side ? -1.0 : 1.0;
            for (double d = 1.0; d <= 3.0; d += 0.5) {
                double px = mx + nx * sgn * d, py = my + ny * sgn * d;
                if (px < 0 || py < 0 || px > OUT_W - 1 || py > OUT_H - 1) continue;
                if (!pointInPolyF(px, py, xs, ys)) continue;
                int v = img[(int)(py + 0.5) * OUT_W + (int)(px + 0.5)];
                if (v < ISO) black++; else white++;
                break;
            }
        }
        if (white + black >= 10) break;
    }
    if (white + black == 0) return 1;
    return white >= black ? 1 : 0;
}

// 输出一条折线
static void writePoly(const vector<int> &xs, const vector<int> &ys,
                      OutFile &fX, OutFile &fY,
                      int &totalVerts, int &totalPolys, int &framePolys)
{
    for (size_t j = 0; j < xs.size(); ++j) {
        if (j) { fX.comma(); fY.comma(); }
        fX.putInt(xs[j]);
        fY.putInt(ys[j]);
    }
    fX.nl();
    fY.nl();
    totalVerts += (int)xs.size();
    totalPolys++;
    framePolys++;
}

// 把整帧的轮廓整理成"纯白区域"后输出
static void emitWhiteRegions(const unsigned char *img, const vector<Region> &regs,
                             OutFile &fX, OutFile &fY,
                             int &totalVerts, int &totalPolys, int &framePolys)
{
    int m = (int)regs.size();
    if (m == 0) return;

    // ---- 父子关系：被谁最小地包含 ----
    vector<int> parent(m, -1);
    for (int i = 0; i < m; ++i) {
        double ai = fabs(regs[i].area);
        double best = 1e30;
        int bp = -1;
        for (int j = 0; j < m; ++j) {
            if (j == i) continue;
            double aj = fabs(regs[j].area);
            if (aj <= ai) continue;
            if (!pointInPoly(regs[i].xs[0], regs[i].ys[0], regs[j].xs, regs[j].ys)) continue;
            if (aj < best) { best = aj; bp = j; }
        }
        parent[i] = bp;
    }

    // ---- 只输出内部是白的区域 ----
    for (int i = 0; i < m; ++i) {
        if (!regs[i].color) continue;

        // 收集夹在里面的黑色区域
        vector<int> kid;
        for (int j = 0; j < m; ++j)
            if (parent[j] == i && !regs[j].color) kid.push_back(j);

        const vector<int> &oxs = regs[i].xs, &oys = regs[i].ys;
        int on = (int)oxs.size();

        // 每个黑洞：找最近的点对当缝的挂接点，并把方向转成与父轮廓相反
        vector<int> slot(kid.size(), 0);
        vector<vector<int> > bxs(kid.size()), bys(kid.size());
        for (size_t t = 0; t < kid.size(); ++t) {
            const Region &c = regs[kid[t]];
            int cn = (int)c.xs.size() - 1;          // 不含重复末点的点数
            if (cn < 3) { slot[t] = -1; continue; }

            double bd = 1e30;
            int bi = 0, bj = 0;
            for (int a = 0; a + 1 < on; ++a)
                for (int b = 0; b < cn; ++b) {
                    double dx = (double)oxs[a] - c.xs[b];
                    double dy = (double)oys[a] - c.ys[b];
                    double d = dx * dx + dy * dy;
                    if (d < bd) { bd = d; bi = a; bj = b; }
                }
            slot[t] = bi;

            bool rev = (c.area * regs[i].area > 0);  // 同向就反过来，非零规则才挖得掉
            bxs[t].resize(cn + 1);
            bys[t].resize(cn + 1);
            for (int q = 0; q < cn; ++q) {
                int idx = rev ? (((bj - q) % cn) + cn) % cn : (bj + q) % cn;
                bxs[t][q] = c.xs[idx];
                bys[t][q] = c.ys[idx];
            }
            bxs[t][cn] = bxs[t][0];
            bys[t][cn] = bys[t][0];
        }

        // 沿父轮廓走，遇到挂接点就插一段"去黑洞、绕一圈、原路返回"
        vector<int> outx, outy;
        outx.reserve(on + 32);
        outy.reserve(on + 32);
        for (int a = 0; a < on; ++a) {
            outx.push_back(oxs[a]);
            outy.push_back(oys[a]);
            for (size_t t = 0; t < kid.size(); ++t) {
                if (slot[t] != a) continue;
                for (size_t q = 0; q < bxs[t].size(); ++q) {
                    outx.push_back(bxs[t][q]);
                    outy.push_back(bys[t][q]);
                }
                outx.push_back(oxs[a]);          // 从缝原路回来
                outy.push_back(oys[a]);
            }
        }
        if (outx.size() < 3) continue;
        if (!(outx.front() == outx.back() && outy.front() == outy.back())) {
            outx.push_back(outx.front());
            outy.push_back(outy.front());
        }
        writePoly(outx, outy, fX, fY, totalVerts, totalPolys, framePolys);
    }
}

/* ---------------- 主程序 ---------------- */

int main(int argc, char **argv)
{
    int fps      = FPS;
    double eps   = SIMPLIFY_EPS;
    int maxFrames = MAX_FRAMES;

    // 命令行参数：程序名 [帧率] [抽稀容差] [最多帧数]
    if (argc >= 2) { int v = atoi(argv[1]);    if (v > 0) fps = v; }
    if (argc >= 3) { double v = atof(argv[2]); if (v > 0) eps = v; }
    if (argc >= 4) { int v = atoi(argv[3]);    if (v > 0) maxFrames = v; }

    printf("=== video -> line segments ===\n");
    printf("video : %s\n", VIDEO_PATH);

    // 没有参数就交互式问帧率，回车用默认值
    if (argc < 2) {
        char line[64];
        printf("read fps (1-120), press Enter for %d : ", FPS);
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin)) {
            int v = atoi(line);
            if (v > 0) fps = v;
        }
    }
    if (fps < 1) fps = 1;
    if (fps > 120) fps = 120;

#ifdef _WIN32
    if (!fileExists(VIDEO_PATH)) {
        printf("!! video not found: %s\n", VIDEO_PATH);
        printf("   (edit VIDEO_PATH at the top of the source, or put the video here)\n");
        return -1;
    }
    string vabs  = absPath(VIDEO_PATH);
    string vpath = cmdPath(vabs.c_str());
    string fabs  = ffmpegPath();          // 已经是绝对路径
    string fpath = cmdPath(fabs.c_str());

    printf("ffmpeg: %s\n", fabs.c_str());
    if (!fileExists(fabs.c_str())) {
        printf("!! 找不到 ffmpeg：%s\n", fabs.c_str());
        printf("   把源码顶部的 FFMPEG_EXE 改成完整路径，或把 ffmpeg.exe 放到视频同目录。\n");
        return -1;
    }

    // 注意：cmd.exe 有一条规定——命令以引号开头时，它会把首尾引号都吃掉，
    // 结果整行被当成一个程序名（就是"不是内部或外部命令"）。
    // 所以路径没有空格时绝对不要加引号；只有含空格才加，并且把整条命令再包一层引号给它剥。
    bool needQ = (fpath.find(' ') != string::npos);
    string exeArg = needQ ? ("\"" + fpath + "\"") : fpath;

    char body[2048];
    snprintf(body, sizeof(body),
             "%s -hide_banner -loglevel error -nostdin -i \"%s\" -an "
             "-vf \"fps=%d,scale=%d:%d:flags=bicubic\" -pix_fmt gray -f rawvideo -",
             exeArg.c_str(), vpath.c_str(), fps, OUT_W, OUT_H);

    string cmdStr = body;
    if (needQ) cmdStr = "\"" + cmdStr + "\"";

    FILE *pipe = _popen(cmdStr.c_str(), "rb");
    if (!pipe) {
        printf("!! cannot start ffmpeg. check FFMPEG_EXE / PATH.\n");
        printf("   cmd: %s\n", cmdStr.c_str());
        return -1;
    }
#else
    printf("!! windows only\n");
    return -1;
#endif

    printf("size  : %d x %d, fps=%d, iso=%d, eps=%.2f%s\n",
           OUT_W, OUT_H, fps, ISO, eps,
           maxFrames > 0 ? " (limited)" : "");

    // ---- 4 个输出文件 ----
    // 注意：这里先不打开！等真的从 ffmpeg 读到第一帧再打开。
    // 否则 ffmpeg 一旦没跑起来，打开动作就会把上一次生成的数据截断成空文件。
    OutFile fX, fY, fFrame;
    bool filesReady = false;

    vector<unsigned char> gray((size_t)OUT_W * OUT_H);
    vector<unsigned char> blurA((size_t)OUT_W * OUT_H);
    vector<unsigned char> blurB((size_t)OUT_W * OUT_H);
    vector<PolyRec> polys;
    vector<Region>  regs;      // 本帧所有轮廓（整理成纯色区域前）
    vector<int> rx, ry;
    vector<char> keep;

    int totalVerts = 0;    // 顶点总数
    int totalPolys = 0;    // 折线总数
    int frameCnt   = 0;
    int zeroFrames = 0;

    const size_t frameBytes = (size_t)OUT_W * OUT_H;

    while (true) {
        size_t got = fread(&gray[0], 1, frameBytes, pipe);
        if (got < frameBytes) break;

        // 第一帧到手，说明 ffmpeg 正常工作，这时才创建/截断输出文件
        if (!filesReady) {
            if (!fX.open(OUT_X) || !fY.open(OUT_Y) || !fFrame.open(OUT_FRAME)) {
                printf("!! cannot create output files in current directory\n");
#ifdef _WIN32
                _pclose(pipe);
#endif
                return -1;
            }
            filesReady = true;
        }

        // ---- 3x3 [1 2 1] 高斯模糊，压掉压缩噪点 ----
        const unsigned char *img = &gray[0];
        for (int pass = 0; pass < BLUR_TIMES; ++pass) {
            for (int y = 0; y < OUT_H; ++y) {          // 横向
                const unsigned char *src = img + (size_t)y * OUT_W;
                unsigned char *dst = &blurA[(size_t)y * OUT_W];
                for (int x = 0; x < OUT_W; ++x) {
                    int xm = x > 0 ? x - 1 : 0;
                    int xp = x < OUT_W - 1 ? x + 1 : OUT_W - 1;
                    dst[x] = (unsigned char)((src[xm] + 2 * src[x] + src[xp] + 2) >> 2);
                }
            }
            for (int y = 0; y < OUT_H; ++y) {          // 纵向
                int ym = y > 0 ? y - 1 : 0;
                int yp = y < OUT_H - 1 ? y + 1 : OUT_H - 1;
                const unsigned char *s0 = &blurA[(size_t)ym * OUT_W];
                const unsigned char *s1 = &blurA[(size_t)y  * OUT_W];
                const unsigned char *s2 = &blurA[(size_t)yp * OUT_W];
                unsigned char *dst = &blurB[(size_t)y * OUT_W];
                for (int x = 0; x < OUT_W; ++x)
                    dst[x] = (unsigned char)((s0[x] + 2 * s1[x] + s2[x] + 2) >> 2);
            }
            img = &blurB[0];
        }

        // ---- 追踪轮廓 ----
        polys.clear();
        regs.clear();
        traceContours(img, polys);

        // ---- 帧索引：本帧第一条折线在 x.txt / y.txt 中的行号（从 1 开始）----
        // 此时已经写出的折线行数就是 totalPolys，所以下一行是 totalPolys+1
        fFrame.putInt((long long)totalPolys + 1);
        fFrame.nl();

        int framePolys = 0;
        for (size_t i = 0; i < polys.size(); ++i) {
            PolyRec &pl = polys[i];
            int n = (int)pl.xs.size();
            if (n < 3) continue;

            // 周长 / 包围盒，过滤噪点小岛
            double per = 0.0;
            float minx = pl.xs[0], maxx = pl.xs[0], miny = pl.ys[0], maxy = pl.ys[0];
            for (int j = 1; j < n; ++j) {
                double dx = pl.xs[j] - pl.xs[j - 1];
                double dy = pl.ys[j] - pl.ys[j - 1];
                per += sqrt(dx * dx + dy * dy);
                if (pl.xs[j] < minx) minx = pl.xs[j];
                if (pl.xs[j] > maxx) maxx = pl.xs[j];
                if (pl.ys[j] < miny) miny = pl.ys[j];
                if (pl.ys[j] > maxy) maxy = pl.ys[j];
            }
            if (per < MIN_POLY_LEN) continue;
            if ((double)(maxx - minx) * (double)(maxy - miny) < MIN_POLY_AREA) continue;

            // 抽稀
            dpSimplify(pl.xs, pl.ys, eps, keep);

            // 取整 + 去掉重复点
            rx.clear(); ry.clear();
            for (int j = 0; j < n; ++j) {
                if (!keep[j]) continue;
                int xi = (int)floor(pl.xs[j] + 0.5);
                int yi = (int)floor(pl.ys[j] + 0.5);
                if (xi < 0) xi = 0; else if (xi > OUT_W - 1) xi = OUT_W - 1;
                if (yi < 0) yi = 0; else if (yi > OUT_H - 1) yi = OUT_H - 1;
                if (!rx.empty() && rx.back() == xi && ry.back() == yi) continue;
                rx.push_back(xi);
                ry.push_back(yi);
            }
            if (rx.size() < 3) continue;

            // 首尾闭合：把第一个点再追加到末尾，画的时候自动连回来，图形是封口的。
            // （少数轮廓本来就已经回到起点，那就不重复加，免得出现长度 0 的线段）
            if (!(rx.front() == rx.back() && ry.front() == ry.back())) {
                rx.push_back(rx.front());
                ry.push_back(ry.front());
            }

            // 先收集，整帧的轮廓都拿到手以后再决定输出哪些
            Region rg;
            rg.xs = rx;
            rg.ys = ry;
            rg.area = polyArea(rx, ry);
            rg.color = innerColorF(img, pl.xs, pl.ys);   // 用原始轮廓判色
            rg.parent = -1;
            regs.push_back(rg);
        }

        if (regs.empty()) {
            // 整帧只有一种颜色（连一条交界线都没有）。纯白就铺一块整屏的白，
            // 纯黑什么都不用画（底图本来就是黑的）。
            if (img[0] >= ISO) {
                vector<int> ux, uy;
                ux.push_back(0);        uy.push_back(0);
                ux.push_back(OUT_W - 1); uy.push_back(0);
                ux.push_back(OUT_W - 1); uy.push_back(OUT_H - 1);
                ux.push_back(0);        uy.push_back(OUT_H - 1);
                ux.push_back(0);        uy.push_back(0);
                writePoly(ux, uy, fX, fY, totalVerts, totalPolys, framePolys);
            }
        } else {
            // 只输出"内部是白"的区域，黑色区域用零宽细缝从中减掉
            emitWhiteRegions(img, regs, fX, fY, totalVerts, totalPolys, framePolys);
        }

        if (framePolys == 0) zeroFrames++;
        frameCnt++;

        if (frameCnt % 100 == 0)
            printf("frame %d, polys=%d, verts=%d\n", frameCnt, totalPolys, totalVerts);

        if (maxFrames > 0 && frameCnt >= maxFrames) break;
    }

#ifdef _WIN32
    _pclose(pipe);
#endif

    if (frameCnt == 0) {
        printf("\n!! ffmpeg 没有输出任何一帧，可能原因：\n");
        printf("   1) ffmpeg 路径不对（源文件顶部 FFMPEG_EXE，或把 ffmpeg.exe 放到视频同目录）\n");
        printf("   2) 视频路径不对（源文件顶部 VIDEO_PATH）\n");
        printf("   把下面这行拿到 cmd 里手动跑一下就知道错在哪：\n");
#ifdef _WIN32
        printf("   %s\n", cmdStr.c_str());
#endif
    }

    // ---- 末尾哨兵：帧索引 末行 = 折线总行数 + 1（最后一条折线的下一行），
    //      这样最后一帧用 下一项-当前项 也算得对
    fFrame.putInt((long long)totalPolys + 1);
    fFrame.nl();

    fX.close(); fY.close(); fFrame.close();

    printf("\n=== done ===\n");
    printf("frames      : %d\n", frameCnt);
    printf("polylines   : %d\n", totalPolys);
    printf("vertices    : %d\n", totalVerts);
    printf("empty frames: %d\n", zeroFrames);
    printf("files       : %s / %s / %s\n", OUT_X, OUT_Y, OUT_FRAME);
    // 只有双击运行（没带参数）才停住窗口，命令行/管道里调用不会被卡住
    if (argc < 2) {
        printf("press Enter to exit...\n");
        fflush(stdout);
        getchar();
    }
    return 0;
}
