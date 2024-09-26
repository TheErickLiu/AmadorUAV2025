import os
import random
import string
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont


shapes = ['circle', 'semicircle', 'quarter_circle', 'triangle', 'rectangle', 'pentagon', 'star', 'cross']
characters = list(string.ascii_uppercase)
font_lib_path = "/Library/Fonts"

def calc_bounding_box(points: list):
    """Return bounding box for a list of points of (x, y)"""
    X = [p[0] for p in points]
    Y = [p[1] for p in points]
    return (min(X), min(Y), max(X), max(Y))

def draw_ellipse(draw, center, size, outline="black", width=10):
    """Draw a circle and return bounding box"""
    size = size // 2
    xy = (center[0] - size, center[1] - size, center[0] + size, center[1] + size)
    draw.ellipse(xy, outline=outline, width=width)
    return xy

def draw_triangle(draw, center, size, outline="black", width=10):
    size = size // 2
    points = [
        (center[0], center[1] - size), 
        (center[0] - size, center[1] + size), 
        (center[0] + size, center[1] + size)
    ]
    draw.polygon(points, outline=outline, width=width)
    return calc_bounding_box(points)

def draw_semicircle(draw, center, size, outline="black", width=10):
    size = size // 2
    xy = (center[0] - size, center[1] - size // 2, center[0] + size, center[1] + size * 3 // 2)
    draw.pieslice(xy, 180, 360, outline=outline, width=width)
    bbox = (center[0] - size, center[1] - size // 2, center[0] + size, center[1] + size // 2)
    return bbox

def draw_quarter_circle(draw, center, size, outline="black", width=10):
    size = size // 2
    xy = (center[0] -  size * 3, center[1] - size * 3, center[0] + size, center[1] + size)
    draw.pieslice(xy, 0, 90, outline=outline, width=width)
    bbox = (center[0] - size, center[1] - size, center[0] + size, center[1] + size)
    return bbox

def draw_rectangle(draw, center, size, outline="black", width=10, hw_ratio=0.66):
    w = size // 2
    h = int(w * hw_ratio)
    xy = (center[0] - w, center[1] - h, center[0] + w, center[1] + h)
    draw.rectangle(xy, outline=outline, width=width)
    return xy

def draw_pentagon(draw, center, size, rotation=0, outline="black", width=10):
    radius = size // 2
    draw.regular_polygon((center[0], center[1], radius), n_sides=5, rotation=rotation, outline=outline, width=width)

    if rotation == 0:
        y_max = center[1] + radius * math.sqrt(3/4)
    else:
        y_max = center[1] + radius

    points = [
        (center[0], center[1] - radius),
        (center[0] - radius // 2, y_max),
        (center[0] + radius // 2, y_max),
        (center[0] - radius, center[1]),
        (center[0] + radius, center[1]),
    ]
    return calc_bounding_box(points)

def draw_star(draw, center, size, outline="black", width=10):
    return draw_pentagon(draw, center, size, rotation=30, outline=outline, width=width)

def draw_cross(draw, center, size, outline="black", width=5):
    size = size // 3
    draw_rectangle(draw, (center[0], center[1]), size * 3, hw_ratio=1/3, outline=outline, width=width)
    draw_rectangle(draw, (center[0], center[1]), size, hw_ratio=3, outline=outline, width=width)
    draw_rectangle(draw, center, size, hw_ratio=1, outline="white", width=10)
    w = size * 3 // 2
    bbox = (center[0] - w, center[1] - w, center[0] + w, center[1] + w)
    return bbox


def draw_shape(draw, shape, center, size, outline="black", width=10):
    """Draw a shape with center and size"""
    if shape == "circle":
        return draw_ellipse(draw, center, size, outline=outline, width=width)
    elif shape == "triangle":
        return draw_triangle(draw, center, size, outline=outline, width=width)
    elif shape == "semicircle":
        return draw_semicircle(draw, center, size, outline=outline, width=width)
    elif shape == "quarter_circle":
        return draw_quarter_circle(draw, center, size, outline=outline, width=width)
    elif shape == "rectangle":
        return draw_rectangle(draw, center, size, outline=outline, width=width)
    elif shape == "pentagon":
        return draw_pentagon(draw, center, size, outline=outline, width=width)
    elif shape == "star":
        return draw_star(draw, center, size, outline=outline, width=width)
    elif shape == "cross":
        return draw_cross(draw, center, size, outline=outline, width=width)
    else:
        print(f"{shape} is not supported. SKIP")


def draw_character(
        draw, character, bbox, center_ratio = 0.5, char_ratio = 0.75, font_type = "Arial Unicode"
    ):
    """Draw the character that fits inside the shape"""

    # Choose a font size that fits inside the shape's bounding box
    font_size = int(min(bbox[2] - bbox[0], bbox[3] - bbox[1]) * char_ratio)
    font = ImageFont.truetype(f"{font_lib_path}/{font_type}.ttf", font_size)
    
    # Calculate text size and position the character inside the shape
    center_x = bbox[0] + (bbox[2] - bbox[0]) // 2
    center_y = bbox[1] + int((bbox[3] - bbox[1]) * center_ratio)
    text_bb = draw.textbbox((center_x, center_y), character, font=font)
    text_len = text_bb[2] - text_bb[0]
    text_height = text_bb[3] - text_bb[1]
    text_left = center_x - text_len // 2
    text_top = center_y - text_height

    # replace with new text bbox
    text_bb = draw.textbbox((text_left, text_top), character, font=font)

    draw.text((text_left, text_top), character, font=font, fill='black')

    return text_bb


def rotate_points(points, angle, center=(0, 0)):
    """Rotates a list of points around a given center"""
    angle = -angle
    cos_theta = np.cos(np.radians(angle))
    sin_theta = np.sin(np.radians(angle))
    rotated_points = []
    for x, y in points:
        x_diff = x - center[0]
        y_diff = y - center[1]
        x_new = x_diff * cos_theta - y_diff * sin_theta
        y_new = x_diff * sin_theta + y_diff * cos_theta
        x_new += center[0]
        y_new += center[1]
        rotated_points.append((x_new, y_new))
    return rotated_points


def rotate_bbox(bbox, angle, img_size):
    """Rotate bounding box by converting the four points of bbox"""
    img_center = (img_size / 2, img_size / 2)
    x_min, y_min, x_max, y_max = bbox
    points = [(x_min, y_min), (x_min, y_max), (x_max, y_min), (x_max, y_max)]
    return calc_bounding_box(rotate_points(points, angle, img_center))


def normalize_bbox(bb, img_size):
    """Normalize bounding box for the YOLO label format"""
    x_center = (bb[0] + bb[2]) / 2.0 / img_size
    y_center = (bb[1] + bb[3]) / 2.0 / img_size
    width = (bb[2] - bb[0]) / img_size
    height = (bb[3] - bb[1]) / img_size
    return x_center, y_center, width, height


def draw_image(
    shape: str, 
    letter: str, 
    image_size: int, 
    shape_size: int, 
    center_offset: int = 0, 
    rotate_angle: float = 0, 
    letter_ratio: float = 0.5, 
    letter_center_ratio: float = 0.5,
    font_type: str = "Arial Unicode",
):
    """Draw a shape with a letter and then rotate"""

    img = Image.new('RGB', (image_size, image_size), color='white')
    draw = ImageDraw.Draw(img)

    # Draw the shape at the center with a random shift
    center = [image_size//2 + center_offset[0], image_size//2 + center_offset[0]]
    shape_bb = draw_shape(draw, shape, center, shape_size)

    # Draw the character with a random size ratio and center ratio
    char_bb = draw_character(
        draw, 
        letter, 
        shape_bb, 
        center_ratio=letter_center_ratio, 
        char_ratio=letter_ratio,
        font_type=font_type,
    )

    # Rotate with given angle
    img = img.rotate(rotate_angle, expand=False, fillcolor='white')
    rotated_shape_bbox = rotate_bbox(shape_bb, rotate_angle, image_size)
    rotated_char_bbox = rotate_bbox(char_bb, rotate_angle, image_size)

    # Convert to YOLO format
    norm_shape_bbox = normalize_bbox(rotated_shape_bbox, image_size)
    norm_char_bbox = normalize_bbox(rotated_char_bbox, image_size)

    return (img, norm_shape_bbox, norm_char_bbox)


def generate_dataset(num_images, image_size, images_dir, labels_dir, font_type="Arial Unicode"):
    for i in range(num_images):
    
        shape_type = random.choice(shapes)
        character = random.choice(characters)
        
        # Draw the shape that fits 50-70% of the image
        shape_size = int(image_size * random.randint(40, 70) // 100)
        
        # Draw the shape at the image center with a random shift
        center_offset = (
            int(np.random.uniform(-image_size//10, image_size//10)),
            int(np.random.uniform(-image_size//10, image_size//10)),
        )

        # Draw the character with a random ratio to the shape 
        char_ratio = random.randint(40, 70) / 100
        if shape_type == "cross":
            char_ratio = random.randint(30, 50) / 100

        # Draw the character at the shape center with a random shift
        center_ratio = random.randint(40, 60) / 100
        if shape_type == "triangle":
            center_ratio = random.randint(60, 80) / 100

        # Random rotate between 0 and 360 degrees
        angle = random.uniform(0, 360)

        # Draw the image
        img, shape_bb, char_bb = draw_image(
            shape_type, character, image_size, shape_size, center_offset, angle, char_ratio, center_ratio
        )

        # Save the image
        img_path = os.path.join(images_dir, f'{i}.png')
        img.save(img_path)

        # Write bounding boxes in YOLO format (class_id x_center y_center width height)
        label_path = os.path.join(labels_dir, f'{i}.txt')
        with open(label_path, 'w') as f:
            shape_class_id = shapes.index(shape_type)
            f.write(f'{shape_class_id} {shape_bb[0]:.6f} {shape_bb[1]:.6f} {shape_bb[2]:.6f} {shape_bb[3]:.6f}\n')
            
            # Class_id for characters starts after shapes
            char_class_id = characters.index(character) + len(shape_type)
            f.write(f'{char_class_id} {char_bb[0]:.6f} {char_bb[1]:.6f} {char_bb[2]:.6f} {char_bb[3]:.6f}\n')


def main():
    
    num_images = 100
    image_size = 512
    images_dir = 'generated_images'
    labels_dir = 'labels'
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)
    generate_dataset(num_images, image_size, images_dir, labels_dir)


if __name__ == "__main__":
    main()