"""验证 Windows PE 依赖：列出导入 DLL（验证"静态编译版本"无第三方 DLL 依赖）。"""
import sys
import pefile


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else r"target\release\aircraft-router-planner.exe"
    pe = pefile.PE(exe, fast_load=True)
    pe.parse_data_directories(directories=[
        pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_IMPORT'],
        pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_DELAY_IMPORT'],
    ])
    dlls = set()
    for entry in getattr(pe, 'DIRECTORY_ENTRY_IMPORT', []) or []:
        dlls.add(entry.dll.decode())
    for entry in getattr(pe, 'DIRECTORY_ENTRY_DELAY_IMPORT', []) or []:
        dlls.add(entry.dll.decode())
    print("imported DLLs:")
    for d in sorted(dlls):
        print("  ", d)
    system = {'KERNEL32.dll', 'KERNELBASE.dll', 'ntdll.dll', 'USER32.dll', 'ADVAPI32.dll',
              'msvcrt.dll', 'VCRUNTIME140.dll', 'VCRUNTIME140_1.dll', 'MSVCP140.dll',
              'ucrtbase.dll', 'api-ms-win-*', 'SHELL32.dll', 'OLE32.dll', 'WS2_32.dll',
              'bcrypt.dll', 'bcryptprimitives.dll', 'SHLWAPI.dll', 'COMDLG32.dll', 'GDI32.dll',
              'IMM32.dll', 'MSIMG32.dll', 'CRYPT32.dll', 'WINMM.dll', 'setupapi.dll',
              'CFGMGR32.dll', 'PSAPI.dll', 'powrprof.dll', 'oleaut32.dll', 'comdlg32.dll',
              'SHCORE.dll'}
    external = [d for d in sorted(dlls) if not any(d.lower().startswith(s.lower().rstrip('*')) for s in system)]
    if external:
        print("EXTERNAL (non-system) DLLs:", external)
        return 1
    print("OK: only system DLLs imported (static Rust linkage, no third-party DLL)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
