
class OBJLoader:
    def __init__(self, filename,z=60):
        self.vertices = []  
        self.groups = []    
        self.faces = []     
        self.smoothings = [] 
        self.zoom = z
        self.load(filename)
        
    def load(self, filename):
        with open(filename, 'r') as file:
            for line in file:
                parts = line.strip().split()
                if not parts:
                    continue
                prefix = parts[0]
                if prefix == 'v':  
                    x, y, z = map(float, parts[1:4])
                    self.vertices.append((x*self.zoom, y, z*self.zoom))

                elif prefix == 'g':  
                    group_name = parts[1] if len(parts) > 1 else 'default'
                    self.groups.append(group_name)                    
                elif prefix == 'f':  
                    face = []
                    for part in parts[1:]:
                        vertex_index = int(part.split('/')[0]) - 1
                        face.append(vertex_index)
                    self.faces.append(face)
                
                elif prefix == 's':  
                    smoothing = parts[1] if len(parts) > 1 else 'off'
                    self.smoothings.append(smoothing)
                    
    def display_data(self):
        print("Vertices:", self.vertices)
        print("Groups:", self.groups)
        print("Faces:", self.faces)
        print("Smoothing:", self.smoothings)


def create_edges_from_faces(faces):
    edges = set()
    for face in faces:
        for i in range(len(face)):
            edge = (face[i], face[(i + 1) % len(face)])
            edges.add(edge)
    return list(edges)

