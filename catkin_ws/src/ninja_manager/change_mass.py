import xml.etree.ElementTree as ET
import sys
import math
import os

def compute_box_inertia(mass, width, height, depth):
    i_xx = (1 / 12) * mass * (height**2 + depth**2)
    i_yy = (1 / 12) * mass * (width**2 + depth**2)
    i_zz = (1 / 12) * mass * (width**2 + height**2)
    return i_xx, i_yy, i_zz

def compute_cylinder_inertia(mass, radius, height):
    i_xx_iyy = (1 / 12) * mass * (3 * radius**2 + height**2)
    i_zz = 0.5 * mass * radius**2
    return i_xx_iyy, i_xx_iyy, i_zz

def update_with_density(file_path, density):
    tree = ET.parse(file_path)
    root = tree.getroot()
    ET.register_namespace('', "http://sdformat.org/sdf/1.6")

    geometry_type = None
    ixx = iyy = izz = mass = 0

    for link in root.iter('link'):
        for collision in link.findall('collision'):
            geom = collision.find('geometry')

            if geom is None:
                continue

            # === Cylinder ===
            cyl = geom.find('cylinder')
            if cyl is not None:
                radius = float(cyl.find('radius').text)
                length = float(cyl.find('length').text)
                volume = math.pi * radius**2 * length
                mass = volume * density
                ixx, iyy, izz = compute_cylinder_inertia(mass, radius, length)
                geometry_type = "cylinder"
                break

            # === Box ===
            box = geom.find('box')
            if box is not None:
                size_str = box.find('size').text
                size_vals = list(map(float, size_str.strip().split()))
                if len(size_vals) != 3:
                    raise ValueError("Box deve avere 3 dimensioni.")
                width, height, depth = size_vals
                volume = width * height * depth
                mass = volume * density
                ixx, iyy, izz = compute_box_inertia(mass, width, height, depth)
                geometry_type = "box"
                break

    if geometry_type is None:
        raise ValueError(f"❌ Geometria non trovata nel file: {file_path}")

    # Inserisce massa e inerzia
    for inertial in root.iter('inertial'):
        mass_element = inertial.find('mass')
        if mass_element is None:
            mass_element = ET.SubElement(inertial, 'mass')
        mass_element.text = f"{mass:.8f}"

        inertia = inertial.find('inertia')
        if inertia is None:
            inertia = ET.SubElement(inertial, 'inertia')

        def set_inertia_tag(tag, value):
            tag_elem = inertia.find(tag)
            if tag_elem is None:
                tag_elem = ET.SubElement(inertia, tag)
            tag_elem.text = f"{value:.8f}"

        set_inertia_tag('ixx', ixx)
        set_inertia_tag('iyy', iyy)
        set_inertia_tag('izz', izz)
        set_inertia_tag('ixy', 0.0)
        set_inertia_tag('ixz', 0.0)
        set_inertia_tag('iyz', 0.0)

    # Salvataggio
    tree.write(file_path, encoding='utf-8', xml_declaration=True)

    print(f"✅ {os.path.basename(file_path)} aggiornato:")
    print(f"    ➤ Tipo: {geometry_type}")
    print(f"    ➤ Volume: {volume:.6f} m³")
    print(f"    ➤ Massa: {mass:.4f} kg")
    print(f"    ➤ Inerzia: ixx={ixx:.6e}, iyy={iyy:.6e}, izz={izz:.6e}")

# MAIN
if __name__ == "__main__":
    ingredients = ['bread', 'meat', 'cheese', 'tomato', 'salad']
    density = 50000  # in kg/m³, regolabile

    for ingredient in ingredients:
        sdf_file = f"ingredients_models/{ingredient}/model.sdf"
        if not os.path.isfile(sdf_file):
            print(f"⚠️  File non trovato: {sdf_file}")
            continue
        try:
            update_with_density(sdf_file, density)
        except Exception as e:
            print(f"❌ Errore in {sdf_file}: {str(e)}")
