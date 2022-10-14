import os
import functools 
import shutil
import requests
from tqdm.auto import tqdm
 
 
def download(url, savepath):
    """
    savepath: filename as absolute path
    Assume: savepath is a valid filepath
    """
    r = requests.get(url, stream=True, allow_redirects=True)
    if r.status_code != 200:
        r.raise_for_status()  # Will only raise for 4xx codes, so...
        raise RuntimeError(f"Request to {url} returned status code {r.status_code}")
    file_size = int(r.headers.get('Content-Length', 0))
 
    abs_path= savepath
    print('saving path', abs_path)

    desc = "(Unknown total file size)" if file_size == 0 else ""
    r.raw.read = functools.partial(r.raw.read, decode_content=True)  # Decompress if needed
    with tqdm.wrapattr(r.raw, "read", total=file_size, desc=desc) as r_raw:
        with open(abs_path, 'wb') as f:
            shutil.copyfileobj(r_raw, f)

    print('download complete')
    return abs_path

def main():
    print('download.py nothing here')

if __name__ == '__main__':
    main()
