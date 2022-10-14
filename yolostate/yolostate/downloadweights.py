import requests
import os


def main():
    URL = "https://pjreddie.com/media/files/yolov3.weights"
    home_path = os.path.expanduser('~')
    install_path = os.path.join(home_path, 'yolo_weights')
    installed = os.path.exists(install_path)

    if not installed:
        with requests.get(URL, stream=True) as r:
            print(f'downloading file: total size: {len(r.content)}')
            downloaded = 0
            with open("yolov3.weights", 'wb') as f:
                for chunk in r.iter_content(chunk_size=1024 * 64 * 16):
                    # writing one chunk at a time to pdf file
                    if chunk:
                        f.write(chunk)
                        downloaded += 1024 * 64 * 16
                        print(f'downloading [% {100 * (downloaded / len(r.content))}]')

        print('download complete')


if __name__ == '__main__':
    main()
