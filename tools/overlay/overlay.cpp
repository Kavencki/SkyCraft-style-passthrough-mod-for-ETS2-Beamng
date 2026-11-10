// overlay.exe - shows the BeamNG car (cabin or whole car) on top of ETS2.
//
// The SkyCraft "hand layer" idea: BeamNG renders everything around the car in one key colour
// (the mod's key box, magenta). This program captures the BeamNG window with Windows Graphics
// Capture, cuts the key colour out, and draws the rest in a transparent, click-through,
// always-on-top window exactly over ETS2's picture. ETS2 itself is not touched.
//
//   overlay [--source BeamNG.drive] [--target "Euro Truck Simulator 2"] [--opacity 255]
//           [--no-resize]   (by default BeamNG's window is sized to match ETS2's picture)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <unknwn.h>
#include <dwmapi.h>
#include <d3d11.h>
#include <dxgi.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Graphics.Capture.h>
#include <winrt/Windows.Graphics.DirectX.Direct3D11.h>
#include <Windows.Graphics.Capture.Interop.h>
#include <windows.graphics.directx.direct3d11.interop.h>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "dwmapi.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "windowsapp.lib")

namespace wgc = winrt::Windows::Graphics::Capture;
namespace wdx = winrt::Windows::Graphics::DirectX;
namespace wd3d = winrt::Windows::Graphics::DirectX::Direct3D11;

struct Find { const wchar_t* text; HWND self; HWND found; };

static BOOL CALLBACK FindProc(HWND h, LPARAM lp) {
    auto* f = reinterpret_cast<Find*>(lp);
    if (h == f->self || !IsWindowVisible(h) || GetWindow(h, GW_OWNER)) return TRUE;
    wchar_t title[512];
    GetWindowTextW(h, title, 512);
    wchar_t cls[128];
    GetClassNameW(h, cls, 128);
    if (!wcscmp(cls, L"ConsoleWindowClass") || !wcscmp(cls, L"CASCADIA_HOSTING_WINDOW_CLASS")) return TRUE;
    if (wcsstr(title, f->text)) { f->found = h; return FALSE; }
    return TRUE;
}

static HWND FindWindowByTitle(const wchar_t* text, HWND self) {
    Find f{text, self, nullptr};
    EnumWindows(FindProc, reinterpret_cast<LPARAM>(&f));
    return f.found;
}

// Client area of a window in screen coordinates.
static RECT ClientScreenRect(HWND h) {
    RECT r{};
    GetClientRect(h, &r);
    POINT tl{r.left, r.top}, br{r.right, r.bottom};
    ClientToScreen(h, &tl);
    ClientToScreen(h, &br);
    return RECT{tl.x, tl.y, br.x, br.y};
}

// GPS: ETS2's own navigation (route advisor) cut out of ETS2's picture and drawn on top of the
// car, so the car's dashboard does not hide it. overlay.ini next to the exe:
//   [gps] enabled=1  source=l t w h  target=l t w h  (fractions of ETS2's picture)  opacity=0..255
// Ctrl+Shift+G shows / hides it.
struct GpsConfig {
    bool enabled = true;
    float src[4] = {0.814f, 0.680f, 0.180f, 0.308f};
    float dst[4] = {0.814f, 0.680f, 0.180f, 0.308f};
    int opacity = 255;
    void Load(const std::wstring& path) {
        wchar_t v[128];
        GetPrivateProfileStringW(L"gps", L"enabled", L"1", v, 128, path.c_str());
        enabled = wcscmp(v, L"0") != 0;
        auto rect = [&](const wchar_t* key, float* r) {
            GetPrivateProfileStringW(L"gps", key, L"", v, 128, path.c_str());
            float t[4];
            if (swscanf_s(v, L"%f %f %f %f", &t[0], &t[1], &t[2], &t[3]) == 4 && t[2] > 0 && t[3] > 0)
                for (int i = 0; i < 4; ++i) r[i] = t[i];
        };
        rect(L"source", src);
        rect(L"target", dst);
        GetPrivateProfileStringW(L"gps", L"opacity", L"255", v, 128, path.c_str());
        opacity = std::max(0, std::min(255, _wtoi(v)));
    }
};

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    if (m == WM_DESTROY) { PostQuitMessage(0); return 0; }
    if (m == WM_NCHITTEST) return HTTRANSPARENT;
    return DefWindowProcW(h, m, w, l);
}

int wmain(int argc, wchar_t** argv) {
    std::wstring sourceTitle = L"BeamNG.drive", targetTitle = L"Euro Truck Simulator 2";
    int opacity = 255;
    bool resize = true;
    for (int i = 1; i < argc; ++i) {
        if (!wcscmp(argv[i], L"--source") && i + 1 < argc) sourceTitle = argv[++i];
        else if (!wcscmp(argv[i], L"--target") && i + 1 < argc) targetTitle = argv[++i];
        else if (!wcscmp(argv[i], L"--opacity") && i + 1 < argc) opacity = _wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--no-resize")) resize = false;
    }
    winrt::init_apartment(winrt::apartment_type::multi_threaded);
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    // Overlay window: layered, click-through, never activated, always on top, not in taskbar.
    WNDCLASSW wc{};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"EuroSpeedOverlay";
    RegisterClassW(&wc);
    HWND overlay = CreateWindowExW(WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST | WS_EX_NOACTIVATE | WS_EX_TOOLWINDOW,
                                   wc.lpszClassName, L"EuroSpeed overlay", WS_POPUP, 0, 0, 16, 16, nullptr, nullptr,
                                   wc.hInstance, nullptr);

    wprintf(L"[overlay] waiting for '%ls' and '%ls' windows...\n", sourceTitle.c_str(), targetTitle.c_str());
    HWND src = nullptr, dst = nullptr;
    while (!src || !dst) {
        src = FindWindowByTitle(sourceTitle.c_str(), overlay);
        dst = FindWindowByTitle(targetTitle.c_str(), overlay);
        if (!src || !dst) Sleep(1000);
    }
    wprintf(L"[overlay] found both windows\n");

    // Match BeamNG's picture size to ETS2's so the two cameras line up pixel for pixel.
    if (resize) {
        RECT t = ClientScreenRect(dst);
        RECT want{0, 0, t.right - t.left, t.bottom - t.top};
        AdjustWindowRectEx(&want, DWORD(GetWindowLongW(src, GWL_STYLE)), FALSE, DWORD(GetWindowLongW(src, GWL_EXSTYLE)));
        SetWindowPos(src, nullptr, 0, 0, want.right - want.left, want.bottom - want.top,
                     SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
        wprintf(L"[overlay] BeamNG picture set to %ldx%ld (ETS2's size)\n", t.right - t.left, t.bottom - t.top);
    }

    // D3D11 device + WinRT wrapper for the capture API.
    winrt::com_ptr<ID3D11Device> d3d;
    winrt::com_ptr<ID3D11DeviceContext> ctx;
    if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, D3D11_CREATE_DEVICE_BGRA_SUPPORT, nullptr, 0,
                                 D3D11_SDK_VERSION, d3d.put(), nullptr, ctx.put()))) {
        wprintf(L"[overlay] cannot create a D3D11 device\n");
        return 1;
    }
    auto dxgi = d3d.as<IDXGIDevice>();
    winrt::com_ptr<::IInspectable> insp;
    winrt::check_hresult(CreateDirect3D11DeviceFromDXGIDevice(dxgi.get(), insp.put()));
    auto device = insp.as<wd3d::IDirect3DDevice>();

    auto interop = winrt::get_activation_factory<wgc::GraphicsCaptureItem, IGraphicsCaptureItemInterop>();
    wgc::GraphicsCaptureItem item{nullptr};
    winrt::check_hresult(interop->CreateForWindow(src, winrt::guid_of<wgc::GraphicsCaptureItem>(), winrt::put_abi(item)));
    auto size = item.Size();
    auto pool = wgc::Direct3D11CaptureFramePool::CreateFreeThreaded(device, wdx::DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, size);
    auto session = pool.CreateCaptureSession(item);
    try { session.IsCursorCaptureEnabled(false); } catch (...) {}
    session.StartCapture();
    wprintf(L"[overlay] capturing BeamNG; showing the car over ETS2 (close this window to stop)\n");

    // GPS: a second capture, of ETS2's window.
    GpsConfig gps;
    {
        wchar_t exe[MAX_PATH];
        GetModuleFileNameW(nullptr, exe, MAX_PATH);
        std::wstring ini = exe;
        ini = ini.substr(0, ini.find_last_of(L"\\/") + 1) + L"overlay.ini";
        gps.Load(ini);
    }
    bool gpsOn = gps.enabled;
    RegisterHotKey(nullptr, 1, MOD_CONTROL | MOD_SHIFT | MOD_NOREPEAT, 'G');
    wgc::GraphicsCaptureItem gpsItem{nullptr};
    wgc::Direct3D11CaptureFramePool gpsPool{nullptr};
    wgc::GraphicsCaptureSession gpsSession{nullptr};
    winrt::Windows::Graphics::SizeInt32 gpsSize{};
    winrt::com_ptr<ID3D11Texture2D> gpsStaging;
    UINT gsW = 0, gsH = 0;
    std::vector<uint32_t> gpsPix;   // the GPS area of ETS2's picture (BGRA), gpsW x gpsH
    std::vector<uint32_t> etsPix;   // ETS2's whole picture (tw x th) while BeamNG is in front
    int gpsW = 0, gpsH = 0;
    try {
        winrt::check_hresult(interop->CreateForWindow(dst, winrt::guid_of<wgc::GraphicsCaptureItem>(), winrt::put_abi(gpsItem)));
        gpsSize = gpsItem.Size();
        gpsPool = wgc::Direct3D11CaptureFramePool::CreateFreeThreaded(device, wdx::DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, gpsSize);
        gpsSession = gpsPool.CreateCaptureSession(gpsItem);
        try { gpsSession.IsCursorCaptureEnabled(false); } catch (...) {}
        gpsSession.StartCapture();
        wprintf(L"[overlay] GPS: ETS2's navigation shown on top of the car (Ctrl+Shift+G hides it; overlay.ini places it)\n");
    } catch (...) {
        gpsPool = nullptr;
        wprintf(L"[overlay] GPS: cannot capture ETS2's window - no GPS\n");
    }

    winrt::com_ptr<ID3D11Texture2D> staging;
    UINT stW = 0, stH = 0;
    HDC screenDC = GetDC(nullptr);
    HDC memDC = CreateCompatibleDC(screenDC);
    HBITMAP dib = nullptr;
    uint32_t* bits = nullptr;
    int dibW = 0, dibH = 0;
    bool shown = false;
    uint64_t frames = 0;

    MSG msg;
    for (;;) {
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) return 0;
            if (msg.message == WM_HOTKEY && msg.wParam == 1) {
                gpsOn = !gpsOn;
                wprintf(L"[overlay] GPS %ls\n", gpsOn ? L"shown" : L"hidden");
            }
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!IsWindow(src) || !IsWindow(dst)) { wprintf(L"[overlay] a game window closed\n"); return 0; }

        // Only over ETS2 while ETS2 is the active window (not over other programs).
        const HWND fg = GetForegroundWindow();
        // BeamNG active (its menu, put over ETS2 by the bridge so the mouse reaches it): the overlay
        // shows ETS2's own picture where BeamNG is key colour, so it looks the same.
        const bool bngFront = fg == src;
        const bool want = (fg == dst || fg == overlay || bngFront) && !IsIconic(dst);
        if (!want) {
            if (shown) { ShowWindow(overlay, SW_HIDE); shown = false; }
            Sleep(50);
            continue;
        }

        auto frame = pool.TryGetNextFrame();
        if (!frame) { Sleep(1); continue; }
        auto csize = frame.ContentSize();
        if (csize.Width != size.Width || csize.Height != size.Height) {
            size = csize;
            pool.Recreate(device, wdx::DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, size);
            continue;
        }
        auto access = frame.Surface().as<::Windows::Graphics::DirectX::Direct3D11::IDirect3DDxgiInterfaceAccess>();
        winrt::com_ptr<ID3D11Texture2D> tex;
        winrt::check_hresult(access->GetInterface(winrt::guid_of<ID3D11Texture2D>(), tex.put_void()));
        D3D11_TEXTURE2D_DESC desc;
        tex->GetDesc(&desc);
        if (!staging || stW != desc.Width || stH != desc.Height) {
            D3D11_TEXTURE2D_DESC sd = desc;
            sd.Usage = D3D11_USAGE_STAGING;
            sd.BindFlags = 0;
            sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
            sd.MiscFlags = 0;
            staging = nullptr;
            winrt::check_hresult(d3d->CreateTexture2D(&sd, nullptr, staging.put()));
            stW = desc.Width;
            stH = desc.Height;
        }
        ctx->CopyResource(staging.get(), tex.get());

        // Where BeamNG's picture (client area) sits inside the captured window image.
        RECT frameRect{};
        DwmGetWindowAttribute(src, DWMWA_EXTENDED_FRAME_BOUNDS, &frameRect, sizeof(frameRect));
        RECT sc = ClientScreenRect(src);
        int cx0 = sc.left - frameRect.left, cy0 = sc.top - frameRect.top;
        int cw = sc.right - sc.left, ch = sc.bottom - sc.top;
        RECT tr = ClientScreenRect(dst);
        int tw = tr.right - tr.left, th = tr.bottom - tr.top;
        if (cw <= 0 || ch <= 0 || tw <= 0 || th <= 0) continue;

        if (!dib || dibW != tw || dibH != th) {
            if (dib) DeleteObject(dib);
            BITMAPINFO bi{};
            bi.bmiHeader.biSize = sizeof(bi.bmiHeader);
            bi.bmiHeader.biWidth = tw;
            bi.bmiHeader.biHeight = -th;
            bi.bmiHeader.biPlanes = 1;
            bi.bmiHeader.biBitCount = 32;
            bi.bmiHeader.biCompression = BI_RGB;
            dib = CreateDIBSection(screenDC, &bi, DIB_RGB_COLORS, reinterpret_cast<void**>(&bits), nullptr, 0);
            SelectObject(memDC, dib);
            dibW = tw;
            dibH = th;
        }

        // GPS: the newest ETS2 frame's GPS area (kept when no new frame came).
        if (!bngFront && !etsPix.empty()) etsPix.clear();
        if ((gpsOn || bngFront) && gpsPool) {
            if (auto gf = gpsPool.TryGetNextFrame()) {
                auto gsz = gf.ContentSize();
                if (gsz.Width != gpsSize.Width || gsz.Height != gpsSize.Height) {
                    gpsSize = gsz;
                    gpsPool.Recreate(device, wdx::DirectXPixelFormat::B8G8R8A8UIntNormalized, 2, gpsSize);
                } else {
                    auto ga = gf.Surface().as<::Windows::Graphics::DirectX::Direct3D11::IDirect3DDxgiInterfaceAccess>();
                    winrt::com_ptr<ID3D11Texture2D> gt;
                    winrt::check_hresult(ga->GetInterface(winrt::guid_of<ID3D11Texture2D>(), gt.put_void()));
                    D3D11_TEXTURE2D_DESC gd;
                    gt->GetDesc(&gd);
                    if (!gpsStaging || gsW != gd.Width || gsH != gd.Height) {
                        D3D11_TEXTURE2D_DESC sd = gd;
                        sd.Usage = D3D11_USAGE_STAGING;
                        sd.BindFlags = 0;
                        sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
                        sd.MiscFlags = 0;
                        gpsStaging = nullptr;
                        winrt::check_hresult(d3d->CreateTexture2D(&sd, nullptr, gpsStaging.put()));
                        gsW = gd.Width;
                        gsH = gd.Height;
                    }
                    ctx->CopyResource(gpsStaging.get(), gt.get());
                    // ETS2's picture (client area) inside its captured window image.
                    RECT gfr{};
                    DwmGetWindowAttribute(dst, DWMWA_EXTENDED_FRAME_BOUNDS, &gfr, sizeof(gfr));
                    const int ox = tr.left - gfr.left, oy = tr.top - gfr.top;
                    const int x0 = ox + int(gps.src[0] * tw), y0 = oy + int(gps.src[1] * th);
                    gpsW = std::max(1, int(gps.src[2] * tw));
                    gpsH = std::max(1, int(gps.src[3] * th));
                    D3D11_MAPPED_SUBRESOURCE gm;
                    if (SUCCEEDED(ctx->Map(gpsStaging.get(), 0, D3D11_MAP_READ, 0, &gm))) {
                        if (gpsOn) {
                        gpsPix.assign(size_t(gpsW) * gpsH, 0);
                        for (int y = 0; y < gpsH; ++y) {
                            const int sy = y0 + y;
                            if (sy < 0 || sy >= int(gsH)) continue;
                            const uint32_t* row = reinterpret_cast<const uint32_t*>(static_cast<const uint8_t*>(gm.pData) + size_t(sy) * gm.RowPitch);
                            for (int x = 0; x < gpsW; ++x) {
                                const int sx = x0 + x;
                                if (sx >= 0 && sx < int(gsW)) gpsPix[size_t(y) * gpsW + x] = row[sx];
                            }
                        }
                        }
                        if (bngFront) {   // ETS2's whole picture, behind the car
                            etsPix.assign(size_t(tw) * th, 0xFF000000u);
                            for (int y = 0; y < th; ++y) {
                                const int sy = oy + y;
                                if (sy < 0 || sy >= int(gsH)) continue;
                                const uint32_t* row = reinterpret_cast<const uint32_t*>(static_cast<const uint8_t*>(gm.pData) + size_t(sy) * gm.RowPitch);
                                const int n = std::min(tw, int(gsW) - ox);
                                if (ox >= 0 && n > 0) memcpy(&etsPix[size_t(y) * tw], row + ox, size_t(n) * 4);
                            }
                        }
                        ctx->Unmap(gpsStaging.get(), 0);
                    }
                }
            }
        }

        D3D11_MAPPED_SUBRESOURCE map;
        if (FAILED(ctx->Map(staging.get(), 0, D3D11_MAP_READ, 0, &map))) continue;
        const uint8_t* img = static_cast<const uint8_t*>(map.pData);
        // Key: magenta-ish pixels (red and blue high, green low) become transparent, with a soft
        // edge; everything else is the car, premultiplied for the layered window.
        for (int y = 0; y < th; ++y) {
            int sy = cy0 + int((int64_t(y) * ch) / th);
            if (sy < 0 || sy >= int(stH)) { memset(bits + size_t(y) * tw, 0, size_t(tw) * 4); continue; }
            const uint8_t* row = img + size_t(sy) * map.RowPitch;
            uint32_t* out = bits + size_t(y) * tw;
            for (int x = 0; x < tw; ++x) {
                int sx = cx0 + int((int64_t(x) * cw) / tw);
                if (sx < 0 || sx >= int(stW)) { out[x] = 0; continue; }
                const uint8_t* p = row + size_t(sx) * 4;  // B G R A
                int b = p[0], g = p[1], r = p[2];
                // How magenta, relative to brightness: haze/bloom make the key pale pink or
                // dark purple, but its hue stays (red ~ blue, green clearly lower).
                const int mx = std::max(r, b);
                int a = 255;
                if (mx > 20 && std::abs(r - b) * 100 < mx * 70) {   // night tonemapping pushes the key towards hot pink
                    const int s = (std::min(r, b) - g) * 255 / mx;
                    // Neutral greys/whites have s ~ 0; even washed-out key pink is clearly above.
                    // Through the windscreen (glass, sun glare) the key is a pale pink, s ~ 30:
                    // fully cut from s 25 on.
                    if (s > 6) a = s >= 25 ? 0 : 255 - (s - 6) * 255 / 19;
                }
                if (a < 255) {                             // remove the magenta spill on edges
                    r = std::min(r, g + 30);
                    b = std::min(b, g + 30);
                }
                a = a * opacity / 255;
                if (a < 255 && !etsPix.empty()) {   // BeamNG in front: ETS2 shows through the key
                    const uint32_t e = etsPix[size_t(y) * tw + x];
                    const int er = (e >> 16) & 255, eg = (e >> 8) & 255, eb = e & 255;
                    out[x] = 0xFF000000u | (uint32_t((r * a + er * (255 - a)) / 255) << 16) | (uint32_t((g * a + eg * (255 - a)) / 255) << 8) | uint32_t((b * a + eb * (255 - a)) / 255);
                    continue;
                }
                out[x] = (uint32_t(a) << 24) | (uint32_t(r * a / 255) << 16) | (uint32_t(g * a / 255) << 8) | uint32_t(b * a / 255);
            }
        }
        ctx->Unmap(staging.get(), 0);

        // GPS on top of everything, scaled into its target area.
        if (gpsOn && !gpsPix.empty()) {
            const int dx0 = int(gps.dst[0] * tw), dy0 = int(gps.dst[1] * th);
            const int dw = std::max(1, int(gps.dst[2] * tw)), dh = std::max(1, int(gps.dst[3] * th));
            const uint32_t ga = uint32_t(gps.opacity);
            for (int y = 0; y < dh; ++y) {
                const int oy = dy0 + y;
                if (oy < 0 || oy >= th) continue;
                const int sy = int((int64_t(y) * gpsH) / dh);
                for (int x = 0; x < dw; ++x) {
                    const int ox = dx0 + x;
                    if (ox < 0 || ox >= tw) continue;
                    const uint32_t p = gpsPix[size_t(sy) * gpsW + int((int64_t(x) * gpsW) / dw)];
                    const uint32_t r = (p >> 16) & 255, g = (p >> 8) & 255, b = p & 255;
                    bits[size_t(oy) * tw + ox] = (ga << 24) | ((r * ga / 255) << 16) | ((g * ga / 255) << 8) | (b * ga / 255);
                }
            }
        }

        POINT dstPt{tr.left, tr.top}, srcPt{0, 0};
        SIZE sz{tw, th};
        BLENDFUNCTION bf{AC_SRC_OVER, 0, 255, AC_SRC_ALPHA};
        UpdateLayeredWindow(overlay, screenDC, &dstPt, &sz, memDC, &srcPt, 0, &bf, ULW_ALPHA);
        if (!shown) {
            SetWindowPos(overlay, HWND_TOPMOST, tr.left, tr.top, tw, th, SWP_NOACTIVATE | SWP_SHOWWINDOW);
            shown = true;
        }
        if (++frames % 600 == 0) wprintf(L"[overlay] %llu frames\n", frames);
    }
}
