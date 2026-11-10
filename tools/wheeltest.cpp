// wheeltest: prints the raw axes of the first game controller (or one whose name contains
// argv[1]) a few times per second, to set up wheel.ini.  Ctrl+C to stop.
#include "../bridge/wheel.h"

int main(int argc, char** argv) {
    es::Wheel w;
    if (!w.Open(argc > 1 ? argv[1] : "")) { printf("no game controller found\n"); return 1; }
    printf("device: %s\n", w.name.c_str());
    const int n = argc > 2 ? atoi(argv[2]) : 1000000;
    for (int i = 0; i < n; ++i) {
        es::WheelAxes a{};
        DWORD b = 0;
        if (w.Read(a, &b)) {
            for (int k = 0; k < 8; ++k) printf("%s=%5ld ", es::kAxisNames[k], a.v[k]);
            printf("buttons=%08lx\n", b);
        } else {
            printf("read failed\n");
        }
        Sleep(250);
    }
    w.Close();
    return 0;
}
