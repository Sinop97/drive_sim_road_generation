from glob import glob
from tqdm import tqdm
import os
import logging
import csv
import cv2
import numpy as np
from blender.renderer.segmentation_colormap import SIGN_TO_COLOR, SIGN_TO_CLASSID
import argparse


_LOGGER = logging.getLogger(__name__)


def match_input_target_files(a, b):
    # match base filenames only as formats differ between label files
    a_mask = set(os.path.basename(fp).split('.')[0] for fp in a)
    b_mask = set(os.path.basename(fp).split('.')[0] for fp in b)

    uncommon_mask = set.symmetric_difference(a_mask, b_mask)
    if len(uncommon_mask) == 0:
        _LOGGER.debug('No mismatch found between input and target files.')
        return a, b
    else:
        _LOGGER.warning("Mismatching Image Ids found: {}".format(', '.join(s for s in uncommon_mask)))
        return [fp for fp in a if "_".join(os.path.basename(fp).split('_')[0:3]) not in uncommon_mask], \
               [fp for fp in b if "_".join(os.path.basename(fp).split('_')[0:3]) not in uncommon_mask]


def convert_dataset_trafficsignid_only(base_path, draw_debug=False, min_pixel_size=50):
    """
    Reads segmentation images and converts them to .csv file with annotations and visualizes it optionally

    Note: makes the assumption that two labels of the same type are not connected to each other, will treat them as
    one label if that happens for some reason
    """
    semseg_image_path = os.path.join(base_path, 'semseg_color')
    instance_image_path = os.path.join(base_path, 'traffic_sign_id')
    gt_path = os.path.join(base_path, 'signs_ground_truth.csv')

    segmentation_images = sorted(glob(os.path.join(semseg_image_path, '*.png'), recursive=False))
    instance_images = sorted(glob(os.path.join(instance_image_path, '*.exr'), recursive=False))

    COLOR_TO_SIGN = {color: name for name, color in SIGN_TO_COLOR.items()}

    segmentation_images, instance_images = match_input_target_files(segmentation_images, instance_images)

    with open(gt_path, 'w') as csvfile:
        gt_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        gt_writer.writerow(['Image Name', 'x1', 'x2', 'y1', 'y2', 'classid'])
        for semantic_name, instance_name in tqdm(zip(segmentation_images, instance_images)):
            semantic_image = cv2.imread(semantic_name)
            instance_image = cv2.imread(instance_name)[..., 0]

            if draw_debug:
                color_image = cv2.imread(os.path.join(base_path, 'rgb',
                                                      os.path.basename(semantic_name)))

            # find unique combinations of traffic sign id labels and cluster them, put bbs around and write
            for traffic_sign_id in np.sort(np.unique(instance_image))[1:]:
                location_mask = (instance_image == traffic_sign_id)
                colors = np.unique(semantic_image[location_mask], axis=0)

                for color in colors:
                    if tuple(color)[::-1] in COLOR_TO_SIGN:
                        traffic_sign = SIGN_TO_CLASSID[COLOR_TO_SIGN[tuple(color)[::-1]]]
                        unique_positions = np.argwhere(location_mask)

                        x1 = np.min(unique_positions[:, 0])
                        x2 = np.max(unique_positions[:, 0])

                        y1 = np.min(unique_positions[:, 1])
                        y2 = np.max(unique_positions[:, 1])

                        if (x2-x1) + (y2-y1) > min_pixel_size:
                            # use instance name to get png of image path
                            gt_writer.writerow([os.path.basename(semantic_name).split('.')[0], x1, x2, y1, y2, traffic_sign])

                            if draw_debug:
                                cv2.rectangle(color_image, (y1, x1), (y2, x2), [int(val) for val in color])
                                cv2.putText(color_image, str(traffic_sign), (y1, x1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            [int(val) for val in color])
                        break  # skip the case where multiple traffic signs are inside one for some reason

            if draw_debug:
                cv2.destroyAllWindows()
                cv2.imshow('Traffic signs in image {}'.format(os.path.basename(semantic_name)), color_image)
                cv2.waitKey(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate phoenix-style annotations from synthetic segmantation & '
                                                 'sign id')
    parser.add_argument('path', type=str, nargs=1,
                        help='Filepath, should contain the \'semseg_color\' and \'traffic_sign_id\' sub-paths')
    args = parser.parse_args()

    # change the pixel size accordingly to remove too small signs (unit is square pixels)
    convert_dataset_trafficsignid_only(args.path[0], draw_debug=True, min_pixel_size=50)
