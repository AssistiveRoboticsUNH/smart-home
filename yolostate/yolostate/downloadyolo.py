import os
import functools 
import shutil
import requests
from tqdm.auto import tqdm
from shr_utils_py.download import download

#TODO: check if saved file is in correct size
 
# fast download link
def main():
    url=URL = "https://pjreddie.com/media/files/yolov3.weights" 
    url="https://universitysystemnh-my.sharepoint.com/:u:/g/personal/ns1254_usnh_edu/EWN4VNfezfNIiN4B-So8jBIByxuaI5ZaxBj-X1I6wAiBcg?download=1"
    home_path = os.path.expanduser('~')
    filename  = URL.split("/")[-1]
    yolo_dir=os.path.join(home_path, '.yolo')
    install_path = os.path.join(yolo_dir, filename)
    installed = os.path.exists(install_path)
    print('installed path: ', install_path, 'installed?', installed)
    if installed:
        print('already installed.')
        return

    if not os.path.exists(yolo_dir):
        os.mkdir(yolo_dir)
    
    savepath= yolo_dir+'/'+filename
    path=download(url, savepath)

if __name__ == '__main__':
    main()
