from PIL import Image, ImageDraw  # PIL 라이브러리에서 Image와 ImageDraw 모듈을 임포트

# 그 후 코드 내용
image_path = "/home/test/astra_ws/map_grid.png"
image = Image.open(image_path)

# Image dimensions
width, height = image.size

# Define the center point as (0,0)
origin_x = 230  # Given x-coordinate for (0,0)
origin_y = 287  # Given y-coordinate for (0,0)

# Define the grid spacing
grid_spacing = 144  # 1 unit = 144 pixels

# Initialize drawing context
draw = ImageDraw.Draw(image)

# Define grid line color
grid_color = (255, 255, 255)  # White for the grid lines

# Draw vertical grid lines
for x in range(origin_x, width, grid_spacing):
    draw.line([(x, 0), (x, height)], fill=grid_color, width=1)
for x in range(origin_x, 0, -grid_spacing):
    draw.line([(x, 0), (x, height)], fill=grid_color, width=1)

# Draw horizontal grid lines
for y in range(origin_y, height, grid_spacing):
    draw.line([(0, y), (width, y)], fill=grid_color, width=1)
for y in range(origin_y, 0, -grid_spacing):
    draw.line([(0, y), (width, y)], fill=grid_color, width=1)

# Save the updated image with the grid
output_grid_image_path = "/home/test/astra_ws/static/custom_grid.png"
image.save(output_grid_image_path)
output_grid_image_path
