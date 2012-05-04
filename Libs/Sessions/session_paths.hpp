#pragma once

#if defined(WINCE)
    #include <shellapi.h>
#endif

namespace session_paths {

static std::wstring get_data_path()
{
    std::wstring data_path;
#if defined(WINCE)
    wchar_t writable_path[MAX_PATH];
    SHGetSpecialFolderPath(0, writable_path, CSIDL_PERSONAL, false); // MyDocuments path
    data_path = writable_path;
#else
    data_path = L"\\My Documents";
#endif
    data_path += L"\\OrionRTK";
    return data_path;
}

}