import argparse
#import time
import os
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt
from numpy import random
from PIL import Image
import random
from matplotlib import pyplot as plt

#TODO cleaner le code un peu, a été refait rapidement dans les 4 jours avant MégaGÉNIALE

def log(text):
    if opt.log:
        print(text)

def createAnotatedPics():
    log("removing backgrounds")
    folder_cible = "./annotated/images/"
    for count, filename in enumerate(os.listdir(folder_cible)):
        output_path = str(count) + ".png"

        # make dire for first annotations
        img_folder_path = "./annotated/images"
        labels_folder_path = "./annotated/labels"
        if not os.path.exists(img_folder_path):
            os.makedirs(img_folder_path)
        if not os.path.exists(labels_folder_path):
            os.makedirs(labels_folder_path)

        print("annotate"+ str(count))
        #annotate
        img = Image.open(img_folder_path + "/" + output_path)
        columnsize,rowsize=img.size
        img = img.convert("RGBA")

        left = 0
        right = 0

        first_pixel = True
        top_found = False
        top = 0
        bot = 0

        for i in range(rowsize):
            for j in range(columnsize):
                pix=img.getpixel((j,i))
                #box is way tighter than transparency check
                if pix[0] > 1 and pix[1] > 1 and pix[2] > 1 and pix[3] > 1:
                    # first pixel is top
                    if not top_found:
                        top = i
                        top_found = True

                    #if we have top keep updating for bottom
                    if top_found:
                        bot = i
        
                    #if first, set left and right
                    if first_pixel:
                        left = j
                        right = j
                        first_pixel = False
        
                    #adjust left and right
                    if not first_pixel:
                        if j < left:
                            left = j
                        if j > right:
                            right = j

        log(top, bot, left, right)
        log("write label")
        label_path = Path(labels_folder_path + "/" + str(count) + ".txt")

        #get center width and height normalised
        final_width = np.abs(left-right)
        final_height = np.abs(bot-top)
        normalized_width = final_width/img.width
        normalized_height = final_height/img.height
        normalized_center = [((final_width/2 + left))/img.width, ((final_height/2 + top))/img.height]

        with label_path.open('w') as file:
            file.write(opt.class_name + " " + str(normalized_center[0]) + " " + str(normalized_center[1]) + " " + str(normalized_width) + " " + str(normalized_height))

def biased_random_edge(size, repetitions=4, power=3.0):
    #génère une valeur random avec biais vers les extérieur TODO biais un peu trop vers la gauche
    values = [random.random() for _ in range(repetitions)]
    biased_value = min(values) if random.random() < 0.5 else max(values)  # Favorise 0 ou 1
    biased_value = biased_value ** power  # Amplifie le biais
    return int(biased_value * size)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--number', type=int, default=200)
    parser.add_argument('--class_name', type=str, default="0")
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--add_rotation', action='store_true')
    parser.add_argument('--create_annotations', action='store_true')
    parser.add_argument('--log', action='store_true')
    opt = parser.parse_args()
    print(opt)

    if opt.create_annotations:
        createAnotatedPics()

    img_folder_path = "./annotated/images"
    labels_folder_path = "./annotated/labels"

    #créer les path pour les nouvelles images augmentées
    newpath_images = "./new_dataset/images"
    if not os.path.exists(newpath_images):
        os.makedirs(newpath_images)

    newpath_labels = "./new_dataset/labels"
    if not os.path.exists(newpath_labels):
        os.makedirs(newpath_labels)

    log("début de l'augmentation")
    for i in range(opt.number):
        print("augmentation: "+str(i))
        #select images random
        img_name = random.choice(os.listdir(img_folder_path))
        img = Image.open(img_folder_path + "/" + img_name)

        labels = []

        label_name, extension = os.path.splitext(os.path.basename(img_folder_path + "/" + img_name))
        label_name = f"{label_name}.txt"

        with open(labels_folder_path + "/" + label_name) as f:
            while True:
                line = f.readline()
                if not line:
                    break
                labels.append(line.split()[1:])

        # Find background
        # TODO Ajouter des images de background
        log("find background image")
        bg_folder_path = "./backgrounds"
        #bg_folder_path = "./coco/coco/images/coco_train2017/train2017"
        bg_name = random.choice(os.listdir(bg_folder_path))
        bg = Image.open(bg_folder_path + "/" + bg_name).convert("RGBA")
        bg_height, bg_width = bg.height, bg.width
        height, width = img.height, img.width
        center = width/2, height/2
        first = True
        while img.height > bg.height-100 or img.width > bg.width-100:
            scaling = 1.5
            if first:
                scaling = random.randint(6, 10)
                first = False
            
            #resize image
            img = img.resize((int(img.width/scaling), int(img.height/scaling)))

            #get infos
            height, width = img.height, img.width
            center = width/2, height/2
            log("resized")

        #x_offset = random.randint(0, bg_width-width)
        #y_offset = random.randint(0, bg_height-height)
        #x_random = min(random.random(), random.random()) if random.random() < 0.5 else max(random.random(), random.random())
        #y_random = min(random.random(), random.random()) if random.random() < 0.5 else max(random.random(), random.random())

        x_offset = biased_random_edge(bg_width - width, repetitions=5, power=3.5)
        y_offset = biased_random_edge(bg_height - height, repetitions=5, power=3.5)

        log("Find new labels")
        # Rotation and find new labels
        new_labels = [] # format xy xy
        angle = random.randint(0, 360)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        image_rotated = img.rotate(angle, fillcolor=(0, 0, 0, 0))
        
        # Devrait toujours en avoir un seul
        for label in labels:   
            box_center_point = [float(labels[0][0])*width, float(labels[0][1])*height]
            box_width = float(labels[0][2])*width
            box_height = float(labels[0][3])*height

            top_left = [box_center_point[0] - box_width/2, box_center_point[1] - box_height/2]
            top_right = [box_center_point[0] + box_width/2, box_center_point[1] - box_height/2]
            bot_left = [box_center_point[0] - box_width/2, box_center_point[1] + box_height/2]
            bot_right = [box_center_point[0] + box_width/2, box_center_point[1] + box_height/2]

            if opt.add_rotation == False:
                new_top_left = np.array([top_left[0] + x_offset, top_left[1] + y_offset])
                new_bot_right = np.array([bot_right[0] + x_offset, bot_right[1] + y_offset])
                new_labels.append([new_top_left, new_bot_right])
                log(new_labels)
                break
            
            #calculs de roation
            corners = [top_left, top_right, bot_left, bot_right]
            corners = np.hstack([corners, np.ones((4, 1))])
            rotated_corners = M.dot(corners.T).T
            x_min = np.min(rotated_corners[:, 0])
            y_min = np.min(rotated_corners[:, 1])
            x_max = np.max(rotated_corners[:, 0])
            y_max = np.max(rotated_corners[:, 1])

            new_top_left = np.array([x_min + x_offset, y_min + y_offset])
            new_bot_right = np.array([x_max + x_offset, y_max + y_offset])
            new_labels.append(new_top_left, new_bot_right)

            if opt.plot:
                plt.gca().add_patch(plt.Rectangle(new_top_left,
                                      new_bot_right[0] - new_top_left[0],
                                      new_bot_right[1] - new_top_left[1],
                                      fill=False, edgecolor='blue', linewidth=2))


        #write labels
        label_path = Path(newpath_labels + "/augmentation" + str(i) + ".txt")

        log("get annotations")
        #get center width and height normalised
        #new_labels[0][0] -> top left
        #new_labels[0][1] -> bot right
        final_width = np.abs((new_labels[0][0][0] - new_labels[0][1][0]))
        final_height = np.abs((new_labels[0][0][1] - new_labels[0][1][1]))
        normalized_width = final_width/bg.width
        normalized_height = final_height/bg.height

        final_top_left = new_labels[0][0]
        final_bot_right = new_labels[0][1]

        normalized_center = [((final_width/2 + final_top_left[0]))/bg.width, ((final_height/2 + final_top_left[1]))/bg.height]

        with label_path.open('w') as file:
            if opt.add_rotation:
                file.write(opt.class_name + " " + str(normalized_center[0])[:10] + " " + str(normalized_center[1])[:10] + " " + str(normalized_width)[:10] + " " + str(normalized_height)[:10])
            else:
                file.write(opt.class_name + " " + str(normalized_center[0])[:10] + " " + str(normalized_center[1])[:10] + " " + str(normalized_width)[:10] + " " + str(normalized_height)[:10])


        bg = bg.convert("RGBA")
        if opt.add_rotation:
            bg.paste(image_rotated, (x_offset,y_offset), image_rotated.convert('RGBA'))
        else:
            bg.paste(img, (x_offset,y_offset), img.convert('RGBA'))

        # Save picture
        bg.save(newpath_images+"/augmentation"+str(i)+".png", format="PNG")

        if opt.plot:
            plt.imshow(bg)
            plt.show()


