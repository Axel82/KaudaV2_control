import os
import struct
import numpy as np
from pyqtgraph.opengl import MeshData

def load_stl_mesh(filename):
    """
    Load a binary STL file and return a pyqtgraph.opengl.MeshData object.
    
    Args:
        filename (str): Path to the STL file.
        
    Returns:
        MeshData: The loaded mesh data ready for GLMeshItem.
        None: If loading fails.
    """
    if not os.path.exists(filename):
        print(f"Error: STL file not found: {filename}")
        # Return an empty mesh to prevent crash, or None to let caller handle
        return MeshData()

    try:
        with open(filename, 'rb') as f:
            # 80 bytes header
            header = f.read(80)
            # 4 bytes plain integer (number of triangles)
            count_data = f.read(4)
            if len(count_data) < 4:
                print(f"Error: STL file too short: {filename}")
                return MeshData()
                
            count = struct.unpack('<I', count_data)[0]
            
            # Each triangle is 50 bytes:
            # - Normal vector (3 floats = 12 bytes)
            # - Vertex 1 (3 floats = 12 bytes)
            # - Vertex 2 (3 floats = 12 bytes)
            # - Vertex 3 (3 floats = 12 bytes)
            # - Attribute byte count (2 bytes)
            
            # Expected size check
            expected_size = 80 + 4 + count * 50
            file_size = os.path.getsize(filename)
            
            if file_size != expected_size:
                print(f"Warning: STL file size mismatch. Expected {expected_size}, got {file_size}. Processing available data.")
            
            # Use numpy to read efficiently
            # We want to skip the normal (12 bytes) and just get the 3 vertices (3*12=36 bytes),
            # then skip attribute count (2 bytes).
            # Total stride = 50 bytes.
            
            # Define exact dtype for binary STL triangle
            # (normal: 3f4, v1: 3f4, v2: 3f4, v3: 3f4, attr: u2)
            stl_dtype = np.dtype([
                ('normal', '<f4', (3,)),
                ('v1', '<f4', (3,)),
                ('v2', '<f4', (3,)),
                ('v3', '<f4', (3,)),
                ('attr', '<u2')
            ])
            
            # Read all triangles
            data = np.fromfile(f, dtype=stl_dtype, count=count)
            
            # Extract vertices
            # data['v1'] has shape (count, 3)
            # We need to stack them: v1, v2, v3 for each triangle
            
            # Stack v1, v2, v3
            # Shape will be (count, 3, 3) -> flatten to (count*3, 3)
            verts = np.concatenate((data['v1'][:, np.newaxis, :], 
                                    data['v2'][:, np.newaxis, :], 
                                    data['v3'][:, np.newaxis, :]), axis=1)
            
            verts = verts.reshape(-1, 3)
            
            # Create faces array: 0,1,2, 3,4,5, ...
            # Since we didn't unique-ify vertices, we just list them sequentially
            # This 'flat' approach is faster and sufficient for visualization
            
            # Although MeshData can actually infer faces if we just provide valid vertexes for individual triangles? 
            # pyqtgraph MeshData(vertexes=...) expects (N, 3) array. 
            # If we don't provide faces, it might not assume triangles.
            # Let's provide explicit faces for N triangles => N faces
            
            # faces = np.arange(count * 3).reshape(-1, 3) 
            # But actually, MeshData implementation allows just vertexes=[N*3, 3] and it treats every 3 as a face 
            # if we don't provide faces? Let's be explicit to be safe.
            
            faces = np.arange(count * 3, dtype=np.uint32).reshape(count, 3)

            return MeshData(vertexes=verts, faces=faces)

    except Exception as e:
        print(f"Error loading STL {filename}: {e}")
        return MeshData()
