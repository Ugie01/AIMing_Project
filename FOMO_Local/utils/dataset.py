from pathlib import Path
from configs.config import IMAGE_EXTENSIONS



class YOLODataset:
    def __init__(
            self,
            image_dir,
            label_dir,
            image_extensions=IMAGE_EXTENSIONS
            # image_extensions=(".jpg", ".jpeg", ".png")
    ):
        """
        YOLO Dataset Loader
        Args:
            image_dir : image 폴더 경로
            label_dir : YOLO txt label 폴더 경로
        """
        self.image_dir = Path(image_dir)
        self.label_dir = Path(label_dir)

        # 지원 이미지 확장자
        self.image_extensions = image_extensions

        # 이미지 목록 생성
        self.images = []

        for ext in self.image_extensions:
            self.images.extend(
                self.image_dir.glob(f"*{ext}")
            )

        # 정렬
        self.images = sorted(self.images)

        print(
            f"[Dataset] {len(self.images)} images loaded"
        )

    def __len__(self):
        """
        Dataset 크기 반환
        """
        return len(self.images)

    def __getitem__(self, index):
        """
        하나의 sample 반환
        return:
        {
            image_path:
                "xxx.jpg",

            labels:
                [
                    {
                        class_id,
                        x,
                        y,
                        w,
                        h
                    }
                ]
        }
        """

        # 이미지 경로
        image_path = self.images[index]

        # 같은 이름 txt 찾기
        label_path = (
            self.label_dir /
            f"{image_path.stem}.txt"
        )

        # YOLO label 읽기
        labels = self.load_label(
            label_path
        )
        sample = {
            "image_path":
                str(image_path),
            "labels":
                labels
        }
        return sample

    def load_label(self, label_path):
        """
        YOLO txt 파일 parsing
        txt:
        class x y w h

        예:
        0 0.52 0.41 0.12 0.20

        return:
        [
            {
                class_id:0,
                x:0.52,
                y:0.41,
                w:0.12,
                h:0.20
            }
        ]
        """

        objects = []

        # txt 없는 경우
        if not label_path.exists():
            return objects

        with open(
                label_path,
                "r"
        ) as file:
            lines = file.readlines()

        for line in lines:
            line = line.strip()

            # 빈 줄 제거
            if not line:
                continue

            values = line.split()

            # YOLO 형식 검증
            if len(values) != 5:
                raise ValueError(
                    f"Invalid YOLO label format : {label_path}"
                )

            # 문자열 -> 숫자 변환
            class_id = int(values[0])
            x = float(values[1])
            y = float(values[2])
            w = float(values[3])
            h = float(values[4])

            obj = {
                "class_id": class_id,
                "x": x,
                "y": y,
                "w": w,
                "h": h
            }
            objects.append(obj)

        return objects