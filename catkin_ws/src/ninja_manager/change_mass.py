import xml.etree.ElementTree as ET
import sys
import math

def compute_box_inertia(mass, width, height, depth):
    i_xx = (1 / 12) * mass * (height**2 + depth**2)
    i_yy = (1 / 12) * mass * (width**2 + depth**2)
    i_zz = (1 / 12) * mass * (width**2 + height**2)
    return i_xx, i_yy, i_zz

def compute_cylinder_inertia(mass, radius, height):
    i_xx_iyy = (1 / 12) * mass * (3 * radius**2 + height**2)
    i_zz = 0.5 * mass * radius**2
    return i_xx_iyy, i_xx_iyy, i_zz

def update_with_density(file_path,density):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Namespace fix (if any)
    ET.register_namespace('', "http://sdformat.org/sdf/1.6")

    # Trova <link>/<inertial>
    for inertial in root.iter('inertial'):
        # Update mass
        mass_element = inertial.find('mass')

        # Trova dimensioni dal cilindro
        geometry_type = None
        ixx = iyy = izz = None  # valori di default

        for link in root.iter('link'):
            for collision in link.findall('collision'):
                geom = collision.find('geometry')

                # === Caso cilindro ===
                cyl = geom.find('cylinder')
                if cyl is not None:
                    radius = float(cyl.find('radius').text)
                    length = float(cyl.find('length').text)
                    vol=pow(radius,2)*math.pi*length
                    ixx, iyy, izz = compute_cylinder_inertia(vol*density, radius, length)
                    geometry_type = "cylinder"
                    break

                # === Caso box ===
                box = geom.find('box')
                if box is not None:
                    size_str = box.find('size').text  # es. "0.1 0.2 0.05"
                    size_vals = list(map(float, size_str.strip().split()))
                    if len(size_vals) != 3:
                        raise ValueError("Il box deve avere tre dimensioni (x y z)!")
                    width, height, depth = size_vals
                    vol=width*height*depth
                    ixx, iyy, izz = compute_box_inertia(vol*density, width, height, depth)
                    geometry_type = "box"
                    break

        # Se non ho trovato nulla
        if geometry_type is None:
            raise ValueError("Diocanestro, non trovo né un cilindro né un box nel file SDF!")

        print(f"✔️ Geometria riconosciuta: {geometry_type}, inerzia aggiornata.")

        # Scrittura dei valori nella sezione inertia
        inertia = inertial.find('inertia')
        inertia.find('ixx').text = f"{ixx:.8f}"
        inertia.find('iyy').text = f"{iyy:.8f}"
        inertia.find('izz').text = f"{izz:.8f}"
        inertia.find('ixy').text = "0.0"
        inertia.find('ixz').text = "0.0"
        inertia.find('iyz').text = "0.0"

    # Salva il nuovo file
    tree.write(file_path, encoding='utf-8', xml_declaration=True)
    if geometry_type == "cylinder":
        print(f"✔️ File aggiornato con massa = {vol*density} kg (cylinder: radius={radius} m, height={length} m)")
    elif geometry_type == "box":
        print(f"✔️ File aggiornato con massa = {vol*density} kg (box: width={width} m, height={height} m, depth={depth} m)")


def update_sdf_inertia(file_path, new_mass):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Namespace fix (if any)
    ET.register_namespace('', "http://sdformat.org/sdf/1.6")

    # Trova <link>/<inertial>
    for inertial in root.iter('inertial'):
        # Update mass
        mass_element = inertial.find('mass')
        mass_element.text = str(new_mass)

        # Trova dimensioni dal cilindro
        geometry_type = None
        ixx = iyy = izz = None  # valori di default

        for link in root.iter('link'):
            for collision in link.findall('collision'):
                geom = collision.find('geometry')

                # === Caso cilindro ===
                cyl = geom.find('cylinder')
                if cyl is not None:
                    radius = float(cyl.find('radius').text)
                    length = float(cyl.find('length').text)
                    ixx, iyy, izz = compute_cylinder_inertia(new_mass, radius, length)
                    geometry_type = "cylinder"
                    break

                # === Caso box ===
                box = geom.find('box')
                if box is not None:
                    size_str = box.find('size').text  # es. "0.1 0.2 0.05"
                    size_vals = list(map(float, size_str.strip().split()))
                    if len(size_vals) != 3:
                        raise ValueError("Il box deve avere tre dimensioni (x y z)!")
                    width, height, depth = size_vals
                    ixx, iyy, izz = compute_box_inertia(new_mass, width, height, depth)
                    geometry_type = "box"
                    break

        # Se non ho trovato nulla
        if geometry_type is None:
            raise ValueError("Diocanestro, non trovo né un cilindro né un box nel file SDF!")

        print(f"✔️ Geometria riconosciuta: {geometry_type}, inerzia aggiornata.")

        # Scrittura dei valori nella sezione inertia
        inertia = inertial.find('inertia')
        inertia.find('ixx').text = f"{ixx:.8f}"
        inertia.find('iyy').text = f"{iyy:.8f}"
        inertia.find('izz').text = f"{izz:.8f}"
        inertia.find('ixy').text = "0.0"
        inertia.find('ixz').text = "0.0"
        inertia.find('iyz').text = "0.0"

    # Salva il nuovo file
    tree.write(file_path, encoding='utf-8', xml_declaration=True)
    if geometry_type == "cylinder":
        print(f"✔️ File aggiornato con massa = {new_mass} kg (cylinder: radius={radius} m, height={length} m)")
    elif geometry_type == "box":
        print(f"✔️ File aggiornato con massa = {new_mass} kg (box: width={width} m, height={height} m, depth={depth} m)")


# Esempio d’uso:
# update_sdf_inertia("modello_bread.sdf", 1.0)

if __name__ == "__main__":
    # MASSE PERSONALIZZATE
    #masses=[0.5,0.5,0.2,0.2,0.1]       # pesi default
    #masses=[1,1,1,1,1]
    #for ingredient,mass in zip(ingredients,masses):
        #sdf_file=f"ingredients_models/{ingredient}/model.sdf"
        #update_sdf_inertia(sdf_file, mass)  
    # BASATO SU DENSITÀ 
    ingredients=['bread','meat','cheese', 'tomato','salad']
    density=50000
    for ingredient in ingredients:
        sdf_file=f"ingredients_models/{ingredient}/model.sdf"
        update_with_density(sdf_file,density)
