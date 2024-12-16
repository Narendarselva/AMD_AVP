//-----------------------------------------------------------------------------
// This file is licensed under the MIT License.
//
// Copyright (c) 2023 by The DiSTI Corporation.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//-----------------------------------------------------------------------------

#pragma once

#ifdef WIN32
#    include <conio.h>
#    include <stdio.h>
#    include <tchar.h>
#    include <windows.h>

#    pragma comment( lib, "user32.lib" )
#else
#    include <errno.h>
#    include <stdio.h>
#    include <stdlib.h>
#    include <string.h>
#    include <sys/ipc.h>
#    include <sys/shm.h>
#    include <sys/types.h>
#endif

// Windows uses a string, while *nix uses a magic number to identify a shared memory region.
// Adjust this if multiple shared memory regions are required.
#ifdef WIN32
TCHAR MEMORY_REGION_NAME[] = TEXT( "MySharedMemoryRegion" );
#else
const key_t MEMORY_REGION_NAME = 0x6868;
#endif

// Add more content to this structure as needed.
struct SharedMemoryStructure
{
    static const int TEXTURE_BUFFER_SIZE = 1024 * 768 * 4;
    unsigned char    textureBuffer[ TEXTURE_BUFFER_SIZE ];
};

class SharedMemoryInterface
{
public:
    static SharedMemoryInterface& Instance()
    {
        static SharedMemoryInterface s_sharedMemoryInterface;

        return s_sharedMemoryInterface;
    }

    SharedMemoryStructure* GetSharedMemory()
    {
#ifdef WIN32
        return (SharedMemoryStructure*)( _pBuf );
#else
        return (SharedMemoryStructure*)( _shmp );
#endif
    }

    SharedMemoryInterface()
#ifdef WIN32
        : _hMapFile( NULL )
        , _pBuf( NULL )
#else
        : _shmid( -1 )
        , _shmp( NULL )
#endif
    {
#ifdef WIN32
        // Try to open the shared region first.
        _hMapFile = OpenFileMapping( FILE_MAP_ALL_ACCESS, FALSE, MEMORY_REGION_NAME );

        // Create the shared region if it doesn't already exist.
        if( NULL == _hMapFile || GetLastError() )
        {
            _hMapFile = CreateFileMapping( INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof( SharedMemoryStructure ), MEMORY_REGION_NAME );
        }

        if( NULL == _hMapFile || GetLastError() )
        {
            std::cerr << "Could not open file mapping object: " << GetLastError() << std::endl;
            std::exit( -1 );
        }

        _pBuf = (LPTSTR)MapViewOfFile( _hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof( SharedMemoryStructure ) );

        if( NULL == _pBuf || GetLastError() )
        {
            std::cerr << "Could not map view of file: " << GetLastError();

            CloseHandle( _hMapFile );
            std::exit( -1 );
        }
#else
	_shmid = ::shmget( MEMORY_REGION_NAME, sizeof( SharedMemoryStructure ), 0644 | IPC_CREAT );

        if( -1 == _shmid )
        {
            std::cerr << "Could not open shared memory: " << strerror( errno ) << std::endl;
            std::exit( -1 );
        }

        // Attach to the segment to get a pointer to it.
        _shmp = ::shmat( _shmid, NULL, 0 );
        if( (void*)-1 == _shmp )
        {
            std::cerr << "Could not attach to shared memory: " << strerror( errno ) << std::endl;
            std::exit( -1 );
        }
#endif
    }

    ~SharedMemoryInterface()
    {
#ifdef WIN32
        UnmapViewOfFile( _pBuf );
        CloseHandle( _hMapFile );
#else
        ::shmdt( _shmp );
        ::shmctl( _shmid, IPC_RMID, 0 );
#endif
    }

protected:
#ifdef WIN32
    HANDLE  _hMapFile;
    LPCTSTR _pBuf;
#else
    int   _shmid;
    void* _shmp;
#endif
};
