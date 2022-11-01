
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using dtPolyRef = System.UInt32;
using dtTileRef = System.UInt32;

namespace RecastSharp
{
    public unsafe class NavMeshContex : IDisposable
    {
        const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
        const int NAVMESHSET_VERSION = 1;

        struct NavMeshSetHeader
        {
            public int magic;
            public int version;
            public int numTiles;
            public dtNavMeshParams params1;
        };

        struct NavMeshTileHeader
        {
            public dtTileRef tileRef;
            public int dataSize;
        };
        
        public unsafe dtNavMesh navMesh;
        public unsafe dtNavMeshQuery navQuery;

        public NavMeshContex()
        {
        }

        dtNavMesh dtAllocNavMesh()
        {
            return new dtNavMesh();
        }

        void memcpy(void* to, void* from, int count)
        {
            Buffer.MemoryCopy(from,to , count, count);
        }
        
        int InitNav(byte* buffer, int n, ref dtNavMesh navMesh)
        {	
            int index = 0;
            // Read header.
            NavMeshSetHeader header = new NavMeshSetHeader();

            int count = sizeof(NavMeshSetHeader);
            if (index + count > n)
            {
                return -1;
            }
            Buffer.MemoryCopy(buffer, &header, count, count);

            index += count;

            if (header.magic != NAVMESHSET_MAGIC)
            {
                return -2;
            }
            if (header.version != NAVMESHSET_VERSION)
            {
                return -3;
            }

            dtNavMesh mesh = dtAllocNavMesh();
            if (mesh == null)
            {
                return -4;
            }
            var status = mesh.init(&header.params1);
            if (DetourCommon.dtStatusFailed(status))
            {
                return -5;
            }

            // Read tiles.
            for (int i = 0; i < header.numTiles; ++i)
            {
                NavMeshTileHeader tileHeader;

                count = sizeof(NavMeshTileHeader);
                if (index + count > n)
                {
                    return -6;
                }
                memcpy(&tileHeader, buffer + index, count);
                index += count;

                if (tileHeader.tileRef == 0 || tileHeader.dataSize == 0)
                    break;

                var data = DetourCommon.dtAlloc(tileHeader.dataSize);
                if (data == null) break;
                // memset(data, 0, tileHeader.dataSize);

                count = tileHeader.dataSize;
                if (index + count > n)
                {
                    return -7;
                }
                memcpy((void*)data, buffer + index, count);
                index += count;

                dtTileRef ret = 0;
                mesh.addTile((byte*)data, tileHeader.dataSize, (int)dtTileFlags.DT_TILE_FREE_DATA, tileHeader.tileRef, ref ret);
            }
            navMesh = mesh;
            return 0;
        }

        public int Init(byte* buffer, int n)
        {
            int ret = InitNav(buffer, n, ref navMesh);

            if (ret != 0)
            {
                return -1;
            }
		          
            navQuery = new dtNavMeshQuery();
            navQuery.init(navMesh, 2048);
            return 0;
        }
	
        public void Dispose()
        {
            //~NavMeshContex()
            if (navQuery != null)
            {
                dtFreeNavMeshQuery(navQuery);
            }
            if (navMesh != null)
            {
                dtFreeNavMesh(navMesh);
            }
        }
        
        void dtFreeNavMesh(dtNavMesh navmesh)
        {
            if (navmesh == null) return;
            navmesh.Dispose();
            //DetourCommon.dtFree((IntPtr)navmesh);
        }
        
        void dtFreeNavMeshQuery(dtNavMeshQuery navmesh)
        {
            if (navmesh == null) return;
            navmesh.Dispose();
            //dtFree(navmesh);
        }
    };
    
    public unsafe class NavMesh
    {
        static NavMesh instance;

        public static NavMesh GetInstace()
        {
            if (instance == null)
            {
                instance = new NavMesh();
            }
            return instance;
        }
        
        Dictionary<int, NavMeshContex> navMeshContexs = new Dictionary<int, NavMeshContex>();
        
        public NavMeshContex New(int id, byte* buffer, int n)
        {
            NavMeshContex navMeshContex1 = new NavMeshContex();
            int ret = navMeshContex1.Init(buffer, n);
	
            if (ret != 0)
            {
                return null;
            }

            navMeshContexs[id] = navMeshContex1;
            return navMeshContex1;
        }

        public NavMeshContex Get(int id)
        {
            if (navMeshContexs.TryGetValue(id, out NavMeshContex value))
                return value;

            return null;
        }

        public void Clear()
        {
            foreach (var item in navMeshContexs)
            {
                item.Value.Dispose();
            }
            navMeshContexs.Clear();
        }

        private NavMesh()
        {
            
        }
    };
    
    public static unsafe class InvokeWarpper
    {
        const int MAX_POLYS = 256;
        const int MAX_SMOOTH = 2048;

        
        public static NavMeshContex RecastGet(int id)
        {
            return NavMesh.GetInstace().Get(id);
        }
        
        public static NavMeshContex RecastLoad(int id,byte[] buffer, int n)
        {
            fixed (byte* buffs = buffer)
            {
                char* buffs1 = (char*)buffs;
                IntPtr ptr = IntPtr.Zero;
                return  NavMesh.GetInstace().New(id, buffs, n);
                //Marshal.StructureToPtr<NavMeshContex>(ret1, ptr, false);
                //return ptr;
            }
        }

        public static void RecastClear()
        {
            NavMesh.GetInstace().Clear();
        }
        
            
        public unsafe static int RecastFind(NavMeshContex navMeshContex, float[] extents, float[] startPos, float[] endPos,
            float[] straightPath)
        {
            if (navMeshContex == null)
            {
                return -1;
            }
            if (startPos == null)
            {
                return -2;
            }
            if (endPos == null)
            {
                return -3;
            }
            if (straightPath == null)
            {
                return -4;
            }
            if (extents == null)
            {
                return -5;
            }

            //char ss[200];
            //int nn = sprintf(ss, "startPos,%f,%f,%f\n", startPos[0], startPos[1], startPos[2]);
            //fwrite(ss, nn, 1, fp);
            //fflush(fp);

            dtPolyRef startRef = 0;
            dtPolyRef endRef = 0;
            float[] startNearestPt = new float[3];
            float[] endNearestPt = new float[3];

            dtQueryFilter filter = new dtQueryFilter();
            filter.setIncludeFlags(0xffff);
            filter.setExcludeFlags(0);
            // Marshal.

            fixed (float* startPos1 = startPos,extents1=extents,endPos1=endPos,startNearestPtr = startNearestPt,endNearestPtr = endNearestPt)
            {
                navMeshContex.navQuery.findNearestPoly(startPos1, extents1, filter, out startRef, startNearestPtr);
                navMeshContex.navQuery.findNearestPoly(endPos1, extents1, filter, out endRef, endNearestPtr);

            }
            
            dtPolyRef[] polys = new dtPolyRef[MAX_POLYS];
            int npolys;
            byte[] straightPathFlags = new byte[MAX_POLYS];
            dtPolyRef[] straightPathPolys = new dtPolyRef[MAX_POLYS];
            int nstraightPath = 0;
            fixed (float* startNearestPtr1 = startNearestPt,endNearestPtr = endNearestPt)
            {
                fixed (uint* polys1 = polys)
                {
                    navMeshContex.navQuery.findPath(startRef, endRef, startNearestPtr1, endNearestPtr, ref filter, polys1, out npolys, MAX_POLYS);    
                }
            }
            

            if (npolys != 0)
            {
                float[] epos1 = new float[3];
                DetourCommon.dtVcopy(epos1, endNearestPt);

                if (polys[npolys - 1] != endRef)
                {
                    fixed (float* endNearestPt1 = endNearestPt, epos11 = epos1)
                    {
                        bool overPoly = false;
                        navMeshContex.navQuery.closestPointOnPoly(polys[npolys - 1], endNearestPt1, epos11, out overPoly);    
                    }
                    
                }

                fixed (float* startNearestPt1 = startNearestPt,endNearestPt1 = endNearestPt,straightPath1=  straightPath)
                {
                    fixed (uint* polys1 = polys,straightPathPolys1 = straightPathPolys)
                    {
                        fixed (byte* straightPathFlags1 = straightPathFlags)
                        {
                            navMeshContex.navQuery.findStraightPath(startNearestPt1, endNearestPt1, polys1, npolys, straightPath1, straightPathFlags1, straightPathPolys1, out nstraightPath, MAX_POLYS, (int)dtStraightPathOptions.DT_STRAIGHTPATH_ALL_CROSSINGS);    
                        }
                        
                    }
                        
                }
                
            }
	
            return nstraightPath;
        }

        public static int RecastFindNearestPoint(NavMeshContex navMeshContex, float[] extents, float[] startPos, float[] nearestPos)
        {
            if (navMeshContex == null)
            {
                return -1;
            }
            if (startPos == null)
            {
                return -2;
            }
            if (nearestPos == null)
            {
                return -3;
            }
            if (extents == null)
            {
                return -5;
            }

            dtPolyRef startRef = 0;

            dtQueryFilter filter = new dtQueryFilter();
            filter.setIncludeFlags(0xffff);
            filter.setExcludeFlags(0);

            fixed (float* startPos1 = startPos,extents1=extents,nearestPos1=nearestPos)
            {
                navMeshContex.navQuery.findNearestPoly(startPos1, extents1, filter, out startRef, nearestPos1);
            }
            
            return (int)startRef;
        }

        // public static int RecastFindRandomPoint(IntPtr navMeshContex, float[] pos)
        // {
        //         
        // }
        
        // public static int RecastFindRandomPointAroundCircle(IntPtr navMeshContex, float[] extents, const float[] centerPos, const float maxRadius, float[] pos)
        // {
        //     
        // }

    }
}