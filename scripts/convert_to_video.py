#!/usr/bin/env python3
import cv2
import os
import sys
import argparse
from datetime import datetime

def convert_images_to_video(input_dir, output_file, fps=30):
    images = []
    for root, dirs, files in os.walk(input_dir):
        for file in files:
            if file.endswith(".jpg") or file.endswith(".png"):
                images.append(os.path.join(root, file))
    
    images.sort() # Sorting by path works because sessions are named by time

    if not images:
        print(f"No images found in {input_dir}")
        return

    # Read first image to get dimensions
    frame = cv2.imread(images[0])
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    print(f"Converting {len(images)} images to video: {output_file}")
    for i, image in enumerate(images):
        video.write(cv2.imread(image))
        if i % 100 == 0:
            print(f"Processed {i}/{len(images)} images...")

    video.release()
    print("Conversion complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a directory of images to a video file.")
    parser.add_argument("input_dir", help="Directory containing the images")
    parser.add_argument("--output", "-o", help="Output video file path (base name)", default=None)
    parser.add_argument("--fps", type=int, default=30, help="Frames per second (default: 30)")

    args = parser.parse_args()

    if not os.path.isdir(args.input_dir):
        print(f"Error: {args.input_dir} is not a directory.")
        sys.exit(1)

    if args.output is None:
        # Generate output name based on directory name
        dir_name = os.path.basename(os.path.normpath(args.input_dir))
        args.output = f"{dir_name}.mp4"

    convert_images_to_video(args.input_dir, args.output, args.fps)
