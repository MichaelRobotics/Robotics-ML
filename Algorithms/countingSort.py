def counting_sort(arr):
    max_val = max(arr)
    count = [0] * (max_val + 1)

    for num in arr:
        count[num] += 1

    sorted_arr = []
    for i, count_i in enumerate(count):
        sorted_arr.extend([i] * count_i)

    return sorted_arr

numbers = [4, 9, 3, 6, 2]
print(counting_sort(numbers))