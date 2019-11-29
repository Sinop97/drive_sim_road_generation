from glob import glob
from tqdm import tqdm
import os
import logging
import csv
import cv2
import numpy as np
from blender.renderer.segmentation_colormap import SIGN_TO_COLOR


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


def convert_dataset_trafficsignid_only(base_path, draw_debug=False):
    """
    Reads segmentation images and converts them to .csv file with annotations and visualizes it optionally

    Note: makes the assumption that two labels of the same type are not connected to each other, will treat them as
    one label if that happens for some reason
    """
    traffic_image_path = os.path.join(base_path, 'semseg_color')
    gt_path = os.path.join(base_path, 'signs_ground_truth.csv')

    segmentation_images = sorted(glob(os.path.join(traffic_image_path, '*.png'), recursive=False))

    with open(gt_path, 'w') as csvfile:
        gt_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        gt_writer.writerow(['Image Name', 'x1', 'x2', 'y1', 'y2', 'classid'])
        for semantic_name in tqdm(segmentation_images):
            # find all traffic sign ID pixels in semantic image
            semantic_image = cv2.imread(semantic_name, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)

            if draw_debug:
                color_image = cv2.imread(os.path.join(base_path, 'rgb',
                                                      os.path.basename(semantic_name).split('.')[0] + '.png'))

            # find unique combinations of traffic sign id labels and cluster them, put bbs around and write
            for traffic_sign, segmentation_color in SIGN_TO_COLOR.items():
                location_mask = np.all(semantic_image[..., :] == segmentation_color, axis=-1)

                ret, labels = cv2.connectedComponents(location_mask.astype(np.uint8))

                for i in range(1, ret):
                    unique_positions = np.argwhere(labels == i)

                    x1 = np.min(unique_positions[:, 0])
                    x2 = np.max(unique_positions[:, 0])
                    y1 = np.min(unique_positions[:, 1])
                    y2 = np.max(unique_positions[:, 1])

                    # use instance name to get png of image path
                    gt_writer.writerow([os.path.basename(semantic_name).split('.')[0], x1, x2, y1, y2, traffic_sign])

                    if draw_debug:
                        print('Drawing ', unique_positions, ' for instance ', os.path.basename(semantic_image))
                        cv2.rectangle(color_image, (y1, x1), (y2, x2), segmentation_color)

            if draw_debug:
                cv2.destroyAllWindows()
                # cv2.imwrite(os.path.join(base_path, 'instance_visualization', os.path.basename(traffic_image)),
                #             color_image)
                cv2.imshow('Traffic signs in image {}'.format(os.path.basename(semantic_name)), color_image)
                cv2.waitKey(0)


if __name__ == '__main__':
    convert_dataset_trafficsignid_only('/home/mykyta/phoenix/drive_sim_road_generation/blender-output',
                                       draw_debug=False)
