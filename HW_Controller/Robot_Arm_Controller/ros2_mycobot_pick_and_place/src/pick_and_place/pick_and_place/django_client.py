
import requests
import base64
import cv2
import numpy as np
import re

# CJ   192.168.0.189
# aa3b 192.168.5.17 

django_url = 'http://192.168.5.17:8000/gwanje/ocr_from_flask_stream/' # 장고 서버가 켜져 있는 ip로 접속

defined_models =['나이키', '아디다스', '뉴발란스', '반스', '컨버스', '푸마', '로투스']
defined_colors =['white', 'black', 'red', 'blue', 'green', 'yellow', 'gray', 'brown', 'pink', 'purple']
defined_sizes = ['230', '235', '240', '245', '250', '255', '260', '265', '270', '275', '280', '285', '290', '12345']

def ask_django_ocr(url, mode):
    try:
        response = requests.post(url)
        response.raise_for_status()
        data = response.json()
    except Exception as e:
        print(f"[ERROR] Django OCR 요청 실패: {e}")
        return None

    if not data or 'results' not in data:
        print("탐지된 텍스트 없음!")
        return None

    word_coords = data['results']

    if mode == 'get_coords':
        return word_coords

    elif mode == 'get_shoe_info':
        tmp_json = {'model': 'None', 'color': 'None', 'size': -1}

        for item in word_coords:
            text = item['text'].strip()

            # 색상 체크
            if text.lower() in defined_colors:
                tmp_json['color'] = text.lower()

            # 모델 체크
            elif text in defined_models:
                tmp_json['model'] = text

            # 사이즈 체크
            elif text.isdigit() and text in defined_sizes:
                tmp_json['size'] = int(text)

        return tmp_json  # ★ 리스트 대신 딕셔너리 바로 반환

    elif mode == 'show_image':
        img_b64 = data.get('image')
        if not img_b64:
            print("이미지 데이터 없음!")
            return None

        img_bytes = base64.b64decode(img_b64)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        cv2.imshow('Received Image', img)
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cv2.destroyAllWindows()
        return None

# if __name__ == '__main__':
    
#     # ask_django_ocr(django_url, 'get_coords')
#     shoe_info = ask_django_ocr(django_url, 'get_shoe_info')
#     # ask_django_ocr(django_url, 'show_image')

#     model = shoe_info['model']
#     color = shoe_info['color']
#     size = shoe_info['size']

#     print(f"{model} || {color} || {size}")



