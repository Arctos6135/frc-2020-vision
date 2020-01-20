from timeit import timeit
from get_target import *
from get_distance import *
from glob import glob
import cv2

paths = glob("*.png")

def benchmark(img_path, number_of_times):
    """
    Benchmarks the algorithms. 
    """
    img = cv2.imread(img_path)
    preprocess_time = timeit(lambda: preprocess(img), number=number_of_times, globals=globals()) / number_of_times
    processed = preprocess(img)
    get_target_time = timeit(lambda: get_target(processed), number=number_of_times, globals=globals()) / number_of_times
    target = get_target(processed)
    get_info_time = timeit(lambda: get_target_info(target), number=number_of_times, globals=globals()) / number_of_times
    total_time = timeit(lambda: get_target_info(get_target(preprocess(img))), number=number_of_times, globals=globals()) / number_of_times
    print(f"""
    preprocess(): {preprocess_time * 1000:.3f}ms
    get_target(): {get_target_time * 1000:.3f}ms
    get_target_info(): {get_info_time * 1000:.3f}ms
    Total: {total_time * 1000:.3f}ms
    """)

if __name__ == "__main__":
    if not paths:
        print("No test images found.")
        exit()
    benchmark(paths[0], 100)
