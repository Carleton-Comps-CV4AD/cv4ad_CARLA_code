#adapted from 

from PIL import Image                                      # (pip install Pillow)
import numpy as np                                         # (pip install numpy)
from skimage import measure                                # (pip install scikit-image)
from shapely.geometry import Polygon, MultiPolygon         # (pip install Shapely)
import os
import json
import os
from pycocotools import mask
import shutil
import random
import math

def create_sub_masks(mask_image, width, height):
    # Initialize a dictionary of sub-masks indexed by RGB colors
    sub_masks = {}
    for x in range(width):
        for y in range(height):
            # Get the RGB values of the pixel
            pixel = mask_image.getpixel((x,y))[:3]
            if pixel[0] in (12, 13, 14, 15, 16, 17, 18, 19):
                # Check to see if we have created a sub-mask...
                pixel_str = str(pixel)
                sub_mask = sub_masks.get(pixel_str)
                if sub_mask is None:
                # Create a sub-mask (one bit per pixel) and add to the dictionary
                    # Note: we add 1 pixel of padding in each direction
                    # because the contours module doesn"t handle cases
                    # where pixels bleed to the edge of the image
                    sub_masks[pixel_str] = Image.new("1", (width+2, height+2))

                # Set the pixel value to 1 (default is 0), accounting for padding
                sub_masks[pixel_str].putpixel((x+1, y+1), 1)

    return sub_masks

def create_sub_mask_annotation(sub_mask):
    # Find contours (boundary lines) around each sub-mask
    # Note: there could be multiple contours if the object
    # is partially occluded. (E.g. an elephant behind a tree)
    contours = measure.find_contours(np.array(sub_mask), 0.5, positive_orientation="low")

    polygons = []
    # segmentations = []
    for contour in contours:
        # Flip from (row, col) representation to (x, y)
        # and subtract the padding pixel
        for i in range(len(contour)):
            row, col = contour[i]
            contour[i] = (col - 1, row - 1)

        # Make a polygon and simplify it
        poly = Polygon(contour)
        poly = poly.simplify(1.0, preserve_topology=False)
        
        if(poly.is_empty):
            # Go to next iteration, dont save empty values in list
            continue

        # we for some reason get some multipolygons whuich makes the later fucntions rbrek so we will check and deconstruct them
        if isinstance(poly, MultiPolygon):
            polygons.extend(poly.geoms)
        else:
            polygons.append(poly)

        # segmentation = np.array(poly.exterior.coords).ravel().tolist()
        # segmentations.append(segmentation)

    segmentation = mask.encode(np.asfortranarray(sub_mask))
    segmentation["counts"] = segmentation["counts"] # .decode("utf-8")
    
    return polygons, segmentation

def create_category_annotation(category_dict):
    category_list = []

    for key, value in category_dict.items():
        category = {
            "supercategory": "actor",
            "id": value,
            "name": key
        } #notice how the super category and nname are the same here
        category_list.append(category)

    return category_list

def create_video_annotation(id:int, name: str, width = 1920, height = 1080):
    video_info = {
        'id': id,
        'name': name,
        'width': width,
        'height': height 
    }
    return video_info

def create_image_annotation(file_name, width, height, image_id, frame_id, video_id):
    images = {
        "file_name": file_name,
        "height": height,
        "width": width,
        "id": image_id,
        'frame_id': frame_id,
        'video_id': video_id
    } # we will need to add video id for our specific use case

    return images

def create_annotation_format(polygon, segmentation, image_id, category_id, annotation_id, video_id, instance_id, im_height, im_width):
    min_x, min_y, max_x, max_y = polygon.bounds
    min_x = max(0, min_x)
    min_y = max(0, min_y)

    max_x = min(im_width, max_x)
    max_y = min(im_height, max_y)

    width = max_x - min_x
    height = max_y - min_y
    bbox = (min_x, min_y, width, height)
    area = polygon.area

    annotation = {
        "id": annotation_id,
        "video_id": video_id,
        "image_id": image_id,
        "category_id": category_id,
        "instance_id": instance_id,
        "bbox": bbox,
        "segmentation": segmentation,
        "area": area,
        "iscrowd": 0,
    }

    return annotation

def get_coco_json_format():
    # Standard COCO format 
    coco_format = {
        "categories": [],
        "videos": [],
        "images": [],
        "annotations": []
    }

    return coco_format

def generate_vis_annotations(path_to_weather_types):
    #we want this fuinction to go through each, weather, each video, and each image 
    # to generate the correstponding annotations which it will dump in a json file at the end

    # so we start with a general path to the main folder and then a list of weather patterns to denote each subfoldr
    # after that we will join those paths and then navigate to the instance_seg subfolder where we will see our pictures
    # then we need to iterate through all of the video folders while adding each video to the videos 
    # for each folder go through all of the images
    # fr each image add to images then go through the pixels to generate annotations for seg mask

    print('Generating annotations...\n\n')

    weathers = os.listdir(path_to_weather_types)
    splits = ['train', 'val']
    annotation_id = 1

    for weather in weathers:

        for split in splits:

            ann = get_coco_json_format()

            category_dict = {
                'pedestrian' : 12,
                'rider' : 13,
                'car' : 14,
                'truck' : 15,
                'bus' : 16,
                'train': 17,
                'motorcycle' : 18,
                'bicycle' : 19
            }

            ann['categories'] = create_category_annotation(category_dict)

            # get all videos from within the weather dir
            weather_path = os.path.join(path_to_weather_types, weather, split,  'instance_seg')
            videos = os.listdir(weather_path)

            for i in range(len(videos)):
                instances = {}
                inst_id = 1
                video_id = i+1
                #get a list of all the image/frames of a video
                image_path = os.path.join(weather_path, videos[i])
                images = os.listdir(image_path)

                #this funcitons assumes standard image  sizes set in the earlier code of 1920x1080
                ann['videos'].append(create_video_annotation(video_id, videos[i]))

                for j in range(len(images)):
                    image_id = video_id * 100000 + (j+1)
                    frame = os.path.join(image_path, images[j])
                    img = Image.open(frame)

                    #add image information to the dictionary
                    ann['images'].append(create_image_annotation(os.path.join(videos[i],images[j]),img.width, img.height, image_id, j, video_id))

                    sub_masks = create_sub_masks(img, img.width, img.height)

                    for k,v in sub_masks.items():
                        polygons, segmentation = create_sub_mask_annotation(v)

                        if polygons != []:
                            if len(polygons) > 1:
                                # print(f"polygons: {polygons}")
                                # print(f"Types in polygons: {[type(p) for p in polygons]}")

                                polygon = MultiPolygon(polygons)
                                # segmentation = segmentations
                            else:
                                polygon = polygons[0]
                                # segmentation = [np.array(polygons[0].exterior.coords).ravel().tolist()] ##i forgot what this id donig

                            cur_instance = "-".join(k.split()[1:])

                            if cur_instance in instances:
                                instance_id = instances[cur_instance]
                            else:
                                instances[cur_instance] = inst_id
                                instance_id = inst_id
                                inst_id += 1

                            # polygon, segmentation, image_id, category_id, annotation_id, video_id, instance_id, im_height, im_width
                            annotation = create_annotation_format(polygon, segmentation, image_id, int(k[1:3]), annotation_id, video_id, instance_id, img.height, img.width)
                            ann['annotations'].append(annotation)
                            annotation_id += 1

            
            #dump the annotations for each weather pattern in a json dictionary
            json_path = os.path.join(path_to_weather_types, weather, split, 'annotations.json')

            with open(json_path, "w") as file:
                json.dump(ann, file, indent=4)

    print('\n\nFinished generating annotations!')



#function to make video folders where necessary
def unwrap_images(path_to_folders):
    folders = os.listdir(path_to_folders)
        # then we go over all the seed folders and make a new one
        # make a new file for the current weather which we will get by splitting the folder name by - and seeing if a folder 
        # having tha name exist, if not make it and give it.

        # now for ech folder, go over rgb, rgb_seg, and instance_seg folders, keep a counter i for the images so that we 
        # can preserve the logic we later use for the videos i.e. the for loop that follows this

        # for each image in each of the above folders, rename it using i and route it ot the above fders so the shit works.
        # rename the images using the counter i and then move it weather/folder/image
    img_no = 1
    file_extension = '.png' # so we can route proper files

    for folder in folders:
        f = folder.split('-')
        weather = f[0]

        coll_folder = os.path.join(path_to_folders, weather)

        if not os.path.exists(coll_folder):
            os.makedirs(coll_folder)
            img_no = 1

        # cameras = ['instance_seg', 'rgb', 'rgb_seg']
        path = os.path.join(path_to_folders, folder, weather)
        cameras = os.listdir(path)

        for camera in cameras:
            os.makedirs(os.path.join(coll_folder, camera), exist_ok=True)


        # under here we want to keep the same value for each of the images that we are moving
        images_path = os.path.join(path, cameras[0])
        image_list = os.listdir(images_path)

        images = [int(img[:-4]) for img in image_list]
        images = sorted(images)

        for img in images:
            for camera in cameras:
                old_img_name = str(img) + file_extension
                new_img_name = str(img_no) + file_extension

                source_path = os.path.join(path, camera, old_img_name)
                destination_img_path = os.path.join(coll_folder, camera, new_img_name)

                # move it to the right folder
                shutil.move(source_path, destination_img_path)

            # change img_no as needed
            if img_no % 18 == 0:
                img_no += 37
            else:
                img_no += 1


        # after this we want to put all of he imaes into their respective video folders
    for folder in folders:
        folder_path = os.path.join(path_to_folders, folder)

        if os.path.exists(folder_path):
            shutil.rmtree(folder_path)




        # at the end of this, we want to clear all of the folders that have the name seed in them.

def make_video_folders(path_to_cameras):
    file_extension = '.png'
    cameras = os.listdir(path_to_cameras)

    img_path = os.path.join(path_to_cameras, cameras[0])
    img_list = os.listdir(img_path)
        
    if img_list[0][:5] != 'video':
        
        # print(imgs_list)
        imgs = [int(img[:-4]) for img in img_list] # need this to sort images in the right order
        imgs = sorted(imgs)
        # print(path_to_images)
        # print(imgs)

        video_counter  = 0

        
        
        for i in range(len(imgs)):
            cur = int(imgs[i])
            prev = int(imgs[i-1])

            if prev != cur-1 or i == 0:
                    video_counter += 1

            # print(prev, cur)

            for camera in cameras:
                # makes a new folder at the beginning adn everytime that we transition to a new sequence

                destination_folder = os.path.join(os.path.join(path_to_cameras, camera), f'video_{video_counter}')
                os.makedirs(destination_folder, exist_ok=True)

                img = str(imgs[i]) + file_extension

                cur_img_path = os.path.join(os.path.join(path_to_cameras, camera), img)
                destination_img_path = os.path.join(destination_folder, img)
                # print(cur_img_path, '\n\n', destination_img_path)

                # move each image to where it'a supposed to be
                shutil.move(cur_img_path, destination_img_path)
        # success moved images
        return 1
    else:
        # videos were already present
        return 0

#imagine this takes us to the foleder with all the images
def enforce_video_organization(weather_path: str):
    print("Enforcing video folder organization\n\n")

    weathers = os.listdir(weather_path)
    # print(weathers)
    for weather in weathers:
        path_to_cameras = os.path.join(weather_path, weather)
        
        done = make_video_folders(path_to_cameras)

        if done:
            print(f"Video folders have been made for {weather}")
        else:
            print(f"Video structure already present for {weather}")

    print("Done! \n\n")


def split_train_val(path_to_weathers):
    # the main idea here is that we will split the cideos into train and val before generating annotations for each individual data split.
    weathers = os.listdir(path_to_weathers)

    cameras = os.listdir(os.path.join(path_to_weathers, weathers[1]))
    # print(cameras)
    video_list = os.listdir(os.path.join(path_to_weathers, weathers[1], cameras[1]))
    # print(video_list)

    random.shuffle(video_list)

    # now we want to split the videos into train and val
    train_num = math.floor(0.8 * len(video_list))
    splits = {}
    splits['train'] = video_list[:train_num]
    splits['val'] = video_list[train_num:]

    for split in splits.keys():
        for video in splits[split]:
            # we want to go through each weather and then make a train folder if it doesn't exist then move the videos there
            for weather in weathers:

                path_to_set = os.path.join(path_to_weathers, weather, split)
                os.makedirs(path_to_set, exist_ok=True)


                for camera in cameras:
                    os.makedirs(os.path.join(path_to_set, camera), exist_ok=True)

                    cur_video_path = os.path.join(path_to_weathers, weather, camera, video)
                    dest_video_path = os.path.join(path_to_set, camera, video)

                    # print(cur_video_path)
                    # print(dest_video_path)

                    shutil.move(cur_video_path, dest_video_path)

    # clean up all empty folders
    for weather in weathers:
        for camera in cameras:
            path_to_rm = os.path.join(path_to_weathers, weather, camera)

            # print(path_to_rm)

            if os.path.exists(path_to_rm):
                # print(f"{path_to_rm} exists and we're removing it.")
                shutil.rmtree(path_to_rm)


                

if __name__ == '__main__':
    path = '/Data/video_data'

    path_test = '/Data/masaddee/data_trial/0126-172926'

    unwrap_images(path)

    #make sure data is organized into folders
    enforce_video_organization(path)

    #generate annotations
    generate_vis_annotations(path)