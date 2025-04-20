import sys
# Windows path (adjust based on your installation location)
freecad_path = r'C:\Users\z00521af\AppData\Local\Programs\FreeCAD 1.0\bin'
if freecad_path not in sys.path:
    sys.path.append(freecad_path)

import FreeCAD as App
import Part
import math
import numpy as np
# from PySide import QtCore


def find_symmetric_face_pairs(shape, tolerance=0.01):
    """Identifies pairs of faces that are symmetric within tolerance"""
    faces = shape.Faces
    face_pairs = []
    
    # Compare each face with all other faces
    for i in range(len(faces)):
        for j in range(i+1, len(faces)):
            face_a = faces[i]
            face_b = faces[j]
            
            # Quick filtering checks
            # Compare areas
            if abs(face_a.Area - face_b.Area) > tolerance:
                continue
                
            # Compare perimeters
            perimeter_a = sum(edge.Length for edge in face_a.Edges)
            perimeter_b = sum(edge.Length for edge in face_b.Edges)
            if abs(perimeter_a - perimeter_b) > tolerance:
                continue
            
            # Check for potential mirror plane
            center_a = face_a.CenterOfMass
            center_b = face_b.CenterOfMass
            
            # Create midpoint between centers
            midpoint = App.Vector((center_a.x + center_b.x)/2,
                                 (center_a.y + center_b.y)/2,
                                 (center_a.z + center_b.z)/2)
            
            # Create normal vector between centers
            normal = center_b.sub(center_a)
            if normal.Length < tolerance:
                continue
            normal.normalize()
            
            # Create mirror transformation
            mirror_transform = App.Matrix()
            mirror_transform.unity()
            mirror_transform.A11 = 1 - 2*normal.x*normal.x
            mirror_transform.A12 = -2*normal.x*normal.y
            mirror_transform.A13 = -2*normal.x*normal.z
            mirror_transform.A14 = 2*normal.x*midpoint.dot(normal)
            mirror_transform.A21 = -2*normal.y*normal.x
            mirror_transform.A22 = 1 - 2*normal.y*normal.y
            mirror_transform.A23 = -2*normal.y*normal.z
            mirror_transform.A24 = 2*normal.y*midpoint.dot(normal)
            mirror_transform.A31 = -2*normal.z*normal.x
            mirror_transform.A32 = -2*normal.z*normal.y
            mirror_transform.A33 = 1 - 2*normal.z*normal.z
            mirror_transform.A34 = 2*normal.z*midpoint.dot(normal)
            
            # Apply mirror transformation to face_a
            mirrored_face = face_a.transformGeometry(mirror_transform)
            
            # Check if the mirrored face coincides with face_b
            if check_faces_coincident(mirrored_face, face_b, tolerance):
                face_pairs.append((i, j, midpoint, normal))
    
    return face_pairs

def check_faces_coincident(face1, face2, tolerance):
    """Check if two faces coincide within tolerance after transformation"""
    # Check if bounding boxes are similar
    bb1 = face1.BoundBox
    bb2 = face2.BoundBox
    
    if (abs(bb1.DiagonalLength - bb2.DiagonalLength) > tolerance):
        return False
    
    # Use vertices and edge points instead of parameter range
    points_to_check = []
    
    # Add vertices
    for vertex in face1.Vertexes:
        points_to_check.append(vertex.Point)
    
    # Sample points on edges
    samples_per_edge = 5
    for edge in face1.Edges:
        for i in range(1, samples_per_edge):
            param = edge.FirstParameter + (i/samples_per_edge) * (edge.LastParameter - edge.FirstParameter)
            try:
                points_to_check.append(edge.valueAt(param))
            except:
                continue
    
    if not points_to_check:
        return False
    
    # Check how many points are close to face2
    match_count = 0
    for point in points_to_check:
        try:
            distance = face2.distToShape(Part.Vertex(point))[0]
            if distance < tolerance:
                match_count += 1
        except:
            pass
    
    # If more than 70% of points match, consider the faces coincident
    return match_count > 0.7 * len(points_to_check)

    
    # # If more than 70% of points match, consider the faces coincident
    # return match_count > 0.7 * samples * samples

def find_significant_mirror_plane(face_pairs, total_faces):
    """Determine if there's a significant mirror plane from the face pairs"""
    if not face_pairs:
        return None, "No symmetrical faces found"
    
    # Group face pairs by similar plane normals
    plane_groups = []
    
    for pair in face_pairs:
        i, j, midpoint, normal = pair
        
        # Find if this normal is similar to an existing group
        found_group = False
        for group in plane_groups:
            group_normal = group[0][3]  # Normal of first pair in group
            if abs(normal.dot(group_normal)) > 0.98:  # Within ~11 degrees
                group.append(pair)
                found_group = True
                break
        
        if not found_group:
            plane_groups.append([pair])
    
    # Find the largest group
    largest_group = max(plane_groups, key=len)
    
    # Calculate average normal and position
    avg_normal = App.Vector(0, 0, 0)
    avg_point = App.Vector(0, 0, 0)
    
    for pair in largest_group:
        avg_normal += pair[3]
        avg_point += pair[2]
    
    avg_normal.normalize()
    avg_point.multiply(1.0/len(largest_group))
    
    # Determine if this is significant symmetry
    unique_faces = set()
    for i, j, _, _ in largest_group:
        unique_faces.add(i)
        unique_faces.add(j)
    
    coverage = len(unique_faces) / total_faces
    
    if coverage < 0.2:
        return None, "No significant symmetry detected"
    elif coverage < 0.6:
        return (avg_point, avg_normal), "Partial symmetry detected"
    else:
        return (avg_point, avg_normal), "Full symmetry detected"

def visualize_mirror_plane(doc, shape, mirror_plane, face_pairs):
    """Visualize the mirror plane and highlighted paired faces"""
    if not mirror_plane:
        return
    
    point, normal = mirror_plane
    
    # Create a planar face to represent the mirror plane
    bbox = shape.BoundBox
    size = max(bbox.XLength, bbox.YLength, bbox.ZLength) * 1.2
    
    # Create orthogonal vectors to the normal
    if abs(normal.x) > 0.1 or abs(normal.y) > 0.1:
        ortho1 = App.Vector(-normal.y, normal.x, 0).normalize()
    else:
        ortho1 = App.Vector(0, -normal.z, normal.y).normalize()
    
    ortho2 = normal.cross(ortho1).normalize()
    
    # Create four corners of the plane
    corners = [
        point.add(ortho1.multiply(size/2).add(ortho2.multiply(size/2))),
        point.add(ortho1.multiply(-size/2).add(ortho2.multiply(size/2))),
        point.add(ortho1.multiply(-size/2).add(ortho2.multiply(-size/2))),
        point.add(ortho1.multiply(size/2).add(ortho2.multiply(-size/2)))
    ]
    
    # Create mirror plane
    wire = Part.makePolygon([corners[0], corners[1], corners[2], corners[3], corners[0]])
    face = Part.Face(wire)
    
    mirror_obj = doc.addObject("Part::Feature", "MirrorPlane")
    mirror_obj.Shape = face
    
    # Safely set ViewObject properties if GUI is available
    if hasattr(mirror_obj, "ViewObject") and mirror_obj.ViewObject is not None:
        mirror_obj.ViewObject.ShapeColor = (0.0, 0.5, 1.0)  # Blue
        mirror_obj.ViewObject.Transparency = 60
    
    # Highlight symmetric face pairs
    paired_faces = []
    for i, j, _, _ in face_pairs:
        paired_faces.append(shape.Faces[i])
        paired_faces.append(shape.Faces[j])
    
    if paired_faces:
        highlight = doc.addObject("Part::Feature", "SymmetricFaces")
        highlight.Shape = Part.makeCompound(paired_faces)
        
        # Safely set ViewObject properties if GUI is available
        if hasattr(highlight, "ViewObject") and highlight.ViewObject is not None:
            highlight.ViewObject.ShapeColor = (1.0, 0.0, 0.0)  # Red
        
    doc.recompute()


# def main(filename):
#     """Main function to detect symmetry in a STEP file"""
#     doc = App.newDocument("SymmetryAnalysis")
#     gui_available = False
#     try:
#         import FreeCADGui
#         if App.GuiUp:
#             gui_available = True
#     except:
#         pass
#     # Import STEP file
#     shape = Part.Shape()
#     shape.read(filename)
    
#     # Create an object to display the shape
#     part_obj = doc.addObject("Part::Feature", "ImportedPart")
#     part_obj.Shape = shape
    
#     # Find symmetric face pairs
#     tolerance = 0.01  # mm
#     face_pairs = find_symmetric_face_pairs(shape, tolerance)
    
#     # Find significant mirror plane
#     mirror_plane, symmetry_status = find_significant_mirror_plane(
#         face_pairs, len(shape.Faces))
    
#     # Print results
#     print(f"Analyzed file: {filename}")
#     print(f"Total faces: {len(shape.Faces)}")
#     print(f"Symmetric face pairs found: {len(face_pairs)}")
#     print(f"Status: {symmetry_status}")
    
#     if mirror_plane:
#         point, normal = mirror_plane
#         print(f"Mirror plane: Point {point}, Normal {normal}")
        
#         # Print paired faces
#         print("Paired faces:")
#         for i, j, _, _ in face_pairs:
#             print(f"Face {i+1} ↔ Face {j+1}")
        
#         if gui_available:
#             visualize_mirror_plane(doc, shape, mirror_plane, face_pairs)
#         else:
#             print("GUI not available - skipping visualization")
    
#     doc.recompute()
#     return doc
def find_all_mirror_planes(face_pairs, total_faces):
    """Identify all potential mirror planes from the face pairs"""
    if not face_pairs:
        return [], "No symmetrical faces found"
    
    # Group face pairs by similar plane normals
    plane_groups = []
    
    for pair in face_pairs:
        i, j, midpoint, normal = pair
        
        # Find if this normal is similar to an existing group
        found_group = False
        for group in plane_groups:
            group_normal = group[0][3]  # Normal of first pair in group
            if abs(normal.dot(group_normal)) > 0.98:  # Within ~11 degrees
                group.append(pair)
                found_group = True
                break
        
        if not found_group:
            plane_groups.append([pair])
    
    # Calculate average planes for each group
    mirror_planes = []
    for group in plane_groups:
        # Only consider groups with at least 2 face pairs
        if len(group) < 2:
            continue
            
        # Calculate average normal and position
        avg_normal = App.Vector(0, 0, 0)
        avg_point = App.Vector(0, 0, 0)
        
        for pair in group:
            avg_normal += pair[3]
            avg_point += pair[2]
        
        avg_normal.normalize()
        avg_point.multiply(1.0/len(group))
        
        # Determine coverage for this plane
        unique_faces = set()
        for i, j, _, _ in group:
            unique_faces.add(i)
            unique_faces.add(j)
        
        coverage = len(unique_faces) / total_faces
        
        # Store the plane information with its coverage and face pairs
        mirror_planes.append({
            'plane': (avg_point, avg_normal),
            'coverage': coverage,
            'pairs': group,
            'face_count': len(unique_faces)
        })
    
    # Sort planes by coverage (highest first)
    mirror_planes.sort(key=lambda x: x['coverage'], reverse=True)
    
    if not mirror_planes:
        return [], "No significant symmetry detected"
    else:
        return mirror_planes, "Multiple symmetry planes detected"

def visualize_all_mirror_planes(doc, shape, mirror_planes):
    """Visualize all detected mirror planes with different colors"""
    if not mirror_planes:
        return
    
    # Define a color palette for different mirror planes
    color_palette = [
        (0.0, 0.5, 1.0),   # Blue
        (0.0, 0.8, 0.3),   # Green
        (1.0, 0.3, 0.0),   # Red-orange
        (0.8, 0.0, 0.8),   # Purple
        (1.0, 0.8, 0.0),   # Yellow
        (0.0, 0.8, 0.8),   # Cyan
        (0.5, 0.3, 0.0),   # Brown
        (0.7, 0.7, 0.7),   # Gray
    ]
    
    bbox = shape.BoundBox
    size = max(bbox.XLength, bbox.YLength, bbox.ZLength) * 1.2
    
    # Create a group to hold all mirror planes
    planes_group = doc.addObject("App::DocumentObjectGroup", "MirrorPlanes")
    
    # Process each mirror plane
    for idx, mirror_plane_data in enumerate(mirror_planes):
        point, normal = mirror_plane_data['plane']
        color_idx = idx % len(color_palette)
        
        # Create orthogonal vectors to the normal
        if abs(normal.x) > 0.1 or abs(normal.y) > 0.1:
            ortho1 = App.Vector(-normal.y, normal.x, 0).normalize()
        else:
            ortho1 = App.Vector(0, -normal.z, normal.y).normalize()
        
        ortho2 = normal.cross(ortho1).normalize()
        
        # Create four corners of the plane
        corners = [
            point.add(ortho1.multiply(size/2).add(ortho2.multiply(size/2))),
            point.add(ortho1.multiply(-size/2).add(ortho2.multiply(size/2))),
            point.add(ortho1.multiply(-size/2).add(ortho2.multiply(-size/2))),
            point.add(ortho1.multiply(size/2).add(ortho2.multiply(-size/2)))
        ]
        
        # Create mirror plane
        wire = Part.makePolygon([corners[0], corners[1], corners[2], corners[3], corners[0]])
        face = Part.Face(wire)
        
        # Add plane with useful name containing coverage info
        coverage_pct = int(mirror_plane_data['coverage'] * 100)
        face_count = mirror_plane_data['face_count']
        mirror_obj = doc.addObject("Part::Feature", f"MirrorPlane_{idx+1}_{coverage_pct}pct_{face_count}faces")
        mirror_obj.Shape = face
        planes_group.addObject(mirror_obj)
        
        # Safely set ViewObject properties if GUI is available
        if hasattr(mirror_obj, "ViewObject") and mirror_obj.ViewObject is not None:
            mirror_obj.ViewObject.ShapeColor = color_palette[color_idx]
            mirror_obj.ViewObject.Transparency = 60
        
        # Add a label to show normal direction
        label_point = point.add(normal.multiply(size/10))
        line = Part.LineSegment(point, label_point).toShape()
        normal_indicator = doc.addObject("Part::Feature", f"Normal_{idx+1}")
        normal_indicator.Shape = line
        planes_group.addObject(normal_indicator)
        
        # Highlight symmetric face pairs for this plane
        paired_faces = []
        for i, j, _, _ in mirror_plane_data['pairs']:
            paired_faces.append(shape.Faces[i])
            paired_faces.append(shape.Faces[j])
        
        if paired_faces:
            highlight = doc.addObject("Part::Feature", f"SymmetricFaces_{idx+1}")
            highlight.Shape = Part.makeCompound(paired_faces)
            planes_group.addObject(highlight)
            
            # Safely set ViewObject properties if GUI is available
            if hasattr(highlight, "ViewObject") and highlight.ViewObject is not None:
                highlight.ViewObject.ShapeColor = color_palette[color_idx]
    
    doc.recompute()

def main(filename):
    """Main function to detect symmetry in a STEP file"""
    doc = App.newDocument("SymmetryAnalysis")
    
    # Import STEP file
    shape = Part.Shape()
    shape.read(filename)
    
    # Create an object to display the shape
    part_obj = doc.addObject("Part::Feature", "ImportedPart")
    part_obj.Shape = shape
    
    # Find symmetric face pairs
    tolerance = 0.01  # mm
    face_pairs = find_symmetric_face_pairs(shape, tolerance)
    
    # Find all mirror planes (not just the major one)
    mirror_planes, symmetry_status = find_all_mirror_planes(
        face_pairs, len(shape.Faces))
    
    # Print results
    print(f"Analyzed file: {filename}")
    print(f"Total faces: {len(shape.Faces)}")
    print(f"Symmetric face pairs found: {len(face_pairs)}")
    print(f"Status: {symmetry_status}")
    print(f"Number of mirror planes detected: {len(mirror_planes)}")
    
    if mirror_planes:
        # Print details for each mirror plane
        for idx, plane_data in enumerate(mirror_planes):
            plane_point, plane_normal = plane_data['plane']
            coverage_pct = plane_data['coverage'] * 100
            print(f"\nMirror Plane #{idx+1}:")
            print(f"  Point: {plane_point}")
            print(f"  Normal: {plane_normal}")
            print(f"  Coverage: {coverage_pct:.1f}%")
            print(f"  Face pairs: {len(plane_data['pairs'])}")
            
            # Optional: Print all paired faces for this plane
            if len(plane_data['pairs']) < 10:  # Only print if not too many
                print("  Paired faces:")
                for i, j, _, _ in plane_data['pairs']:
                    print(f"    Face {i+1} ↔ Face {j+1}")
        
        # Visualize all mirror planes
        visualize_all_mirror_planes(doc, shape, mirror_planes)
    
    doc.recompute()
    
    # Save the document for later viewing in GUI
    output_file = filename.replace(".step", "_symmetry_analysis.FCStd")
    doc.saveAs(output_file)
    print(f"\nSaved analysis to {output_file}")
    print("Open this file in FreeCAD to visualize the results")
    
    return doc

# Example usage
# main("/path/to/GearShaft.step")
