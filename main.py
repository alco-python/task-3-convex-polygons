import math

class Point:
    #вспомогательный класс, реализующий работу с точками

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle=0.0

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __eq__(self, other):
        return math.isclose(self.x, other.x) and math.isclose(self.y, other.y)

    def __str__(self):
        return f"({self.x}, {self.y})"

    def vector_mult(self, other):
        return self.x * other.y - self.y * other.x

    def skalyar_mult(self, other):
        return self.x * other.x + self.y * other.y

    def distance(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class ListNode:
    #вспомогательный класс, реализующий узел связного списка
    def __init__(self, point):
        self.point = point
        self.left = None
        self.right = None


class List:
    #вспомогательный класс, реализующий связный список
    def __init__(self, points):
        self.nodes = []
        self.current = None

        for point in points:
            node = ListNode(point)
            self.nodes.append(node)

        for i in range(len(self.nodes)):
            self.nodes[i].left = self.nodes[(i - 1) % len(self.nodes)]
            self.nodes[i].right = self.nodes[(i + 1) % len(self.nodes)]

        self.current = self.nodes[0] if self.nodes else None

    def remove_node(self, node):
        #удаление узла
        if len(self.nodes) <= 1:
            return None

        node.left.right = node.right
        node.right.left = node.left

        if self.current == node:
            self.current = node.right

        self.nodes.remove(node)
        return self.current

    def __len__(self):
        return len(self.nodes)

class ConvexPolygon:
    def __init__(self, points):
        # принимает список точек
        if len(points) < 3:
            self.is_convex_polygon=0
            self.points = points
            self.angles = []
            print(f"{self} is not convex poligon")
        else:
            self.points = points.copy()
            self.is_convex_polygon = self._check_convexity()
            self.angles = self._calculate_angles()

    def _check_convexity(self):
        #проверка на выпуклость
        n = len(self.points)
        sign = 0
        for i in range(n):
            a = self.points[i]
            b = self.points[(i + 1) % n]
            c = self.points[(i + 2) % n]

            ab = b - a
            bc = c - b

            cross = ab.vector_mult(bc)

            if abs(cross) < 1e-10:
                continue

            if sign == 0:
                sign = 1 if cross > 0 else -1
            elif (cross > 0 and sign == -1) or (cross < 0 and sign == 1):
                return False

        return True

    def _calculate_angles(self):
       #вычисление углов в каждой вершине
        n = len(self.points)
        angles = []

        for i in range(n):
            prev_point = self.points[(i - 1) % n]
            curr_point = self.points[i]
            next_point = self.points[(i + 1) % n]

            v1 = prev_point - curr_point
            v2 = next_point - curr_point

            dot = v1.skalyar_mult(v2)
            cross = v1.vector_mult(v2)

            angle = math.atan2(cross, dot)
            angles.append(angle)

        return angles

    def _update_angles_for_list(self, circular_list):
        #обновление углов в связном списке
        current = circular_list.current
        start_node = current

        while True:
            node = current
            prev_point = node.left.point
            curr_point = node.point
            next_point = node.right.point

            v1 = prev_point - curr_point
            v2 = next_point - curr_point

            dot = v1.skalyar_mult(v2)
            cross = v1.vector_mult(v2)

            curr_point.angle = math.atan2(cross, dot)

            current = current.right
            if current == start_node:
                break
    def area(self):
        #площадь
        if self.is_convex_polygon==1:
            n = len(self.points)
            area = 0.0

            for i in range(n):
                j = (i + 1) % n
                area += self.points[i].x * self.points[j].y
                area -= self.points[j].x * self.points[i].y

            return abs(area) / 2.0
        else:
            print(f"{self} is not convex poligon")



    def perimeter(self):
        #периметр
        if self.is_convex_polygon == 1:
            n = len(self.points)
            perimeter = 0.0

            for i in range(n):
                j = (i + 1) % n
                perimeter += self.points[i].distance(self.points[j])

            return perimeter
        else:
            print(f"{self} is not convex poligon")

    def is_triangle(self, points):
        # проверка на треугольник
        if self.is_convex_polygon == 1:


            if points[0] == points[1] or points[0] == points[2] or points[1] == points[2]:
                return False

            v1 = points[1] - points[0]
            v2 = points[2] - points[0]
            area = abs(v1.vector_mult(v2)) / 2.0

            return area > 1e-10
        else:
            print(f"{self} is not convex poligon")
    def _point_in_polygon(self, point):
        #входит ли точка в многоугольник
        if self.is_convex_polygon == 1:
            n = len(self.points)
            sign = 0

            for i in range(n):
                a = self.points[i]
                b = self.points[(i + 1) % n]

                edge = b - a
                to_point = point - a

                cross = edge.vector_mult(to_point)

                if abs(cross) < 1e-10:
                    continue

                if sign == 0:
                    sign = 1 if cross > 0 else -1
                elif (cross > 0 and sign == -1) or (cross < 0 and sign == 1):
                    return False

            return True
        else:
            print(f"{self} is not convex poligon")
    def contains_polygon(self, other):
        #есть ли общие точки у двух многоугольников
        if self.is_convex_polygon == 1 and other.is_convex_polygon == 1:
            for point in other.points:
                if not self._point_in_polygon(point):
                    return False
            return True
        else:
            print(f"One or two objects are not convex polygons")
    def intersect(self, other):
        #пересечение многоугольников
        if self.is_convex_polygon == 1 and other.is_convex_polygon == 1:
            for point in self.points:
                if other._point_in_polygon(point):
                    return True
            for point in other.points:
                if self._point_in_polygon(point):
                    return True

            return False
        else:
            print(f"One or two objects are not convex polygons")
    def _angle_between_vectors(self, v1, v2):
        #угол между векторами
        dot = v1.skalyar_mult(v2)
        cross = v1.vector_mult(v2)
        return math.atan2(cross, dot)

    def _point_in_triangle(self, point, triangle_points):
        #проверка, лежит ли точка в треугольнике
        if len(triangle_points) != 3:
            return False

        A, B, C = triangle_points

        AB = B - A
        BC = C - B
        CA = A - C

        AP = point - A
        BP = point - B
        CP = point - C

        cross1 = AB.vector_mult(AP)
        cross2 = BC.vector_mult(BP)
        cross3 = CA.vector_mult(CP)

        return (cross1 >= 0 and cross2 >= 0 and cross3 >= 0) or \
            (cross1 <= 0 and cross2 <= 0 and cross3 <= 0)

    def triangulate(self):
        #триангуляция
        if self.is_convex_polygon == 1:


            points_list = List(self.points.copy())

            self._update_angles_for_list(points_list)

            result = []
            current_node = points_list.current

            while len(points_list) > 2:
                temp_triangles = []

                current_point = current_node.point

                # 1. Треугольник с левой стороной
                left_node = current_node.left
                left_left_node = left_node.left

                if left_node.point.angle < math.pi:
                    triangle_points = [current_point, left_node.point, left_left_node.point]
                    if self._is_valid_triangle(triangle_points, points_list):
                        temp_triangles.append((triangle_points, self._angle_variance(triangle_points)))

                # 2. Треугольник с правой стороной
                right_node = current_node.right
                right_right_node = right_node.right

                if right_node.point.angle < math.pi:
                    triangle_points = [current_point, right_node.point, right_right_node.point]
                    if self._is_valid_triangle(triangle_points, points_list):
                        temp_triangles.append((triangle_points, self._angle_variance(triangle_points)))

                # 3. Треугольник с текущей точкой
                if current_point.angle < math.pi:
                    triangle_points = [left_node.point, current_point, right_node.point]
                    if self._is_valid_triangle(triangle_points, points_list):
                        temp_triangles.append((triangle_points, self._angle_variance(triangle_points)))

                if not temp_triangles:
                    current_node = current_node.left
                    continue

                best_triangle = min(temp_triangles, key=lambda x: x[1])[0]

                triangle_polygon = ConvexPolygon(best_triangle)
                result.append(triangle_polygon)

                middle_point = None
                if best_triangle[1] in [node.point for node in points_list.nodes]:
                    middle_point = best_triangle[1]
                elif best_triangle[0] == current_point:
                    middle_point = best_triangle[1] if best_triangle[1] != current_point else best_triangle[2]

                for node in points_list.nodes:
                    if node.point == middle_point:
                        current_node = points_list.remove_node(node)
                        break

                self._update_angles_for_list(points_list)

            return result
        else:
            print(f"{self} is not convex poligon")
    def _is_valid_triangle(self, triangle_points, points_list):
        #проверка треугольника на корректность
        for node in points_list.nodes:
            point = node.point
            if point not in triangle_points and self._point_in_triangle(point, triangle_points):
                return False
        return True

    def _angle_variance(self, triangle_points):
        #разница между минимальным и максимальным углом в треугольнике
        if len(triangle_points) != 3:
            return float('inf')

        A, B, C = triangle_points

        AB = B - A
        AC = C - A
        angle_A = abs(self._angle_between_vectors(AB, AC))

        BA = A - B
        BC = C - B
        angle_B = abs(self._angle_between_vectors(BA, BC))

        CA = A - C
        CB = B - C
        angle_C = abs(self._angle_between_vectors(CA, CB))

        angles = [angle_A, angle_B, angle_C]
        return max(angles) - min(angles)
    def __str__(self):
        points_str = ", ".join(str(p) for p in self.points)
        return f"ConvexPolygon([{points_str}], convex={self.is_convex_polygon})"

    def __repr__(self):
        return self.__str__()


if __name__ == "__main__":
    try:
        square_points = [
            Point(0, 0),
            Point(5, 0),
            Point(5, 5),
            Point(0, 5)
        ]
        square = ConvexPolygon(square_points)
        print(f"Квадрат: {square}")
        print(f"Площадь квадрата: {square.area():.2f}")
        print(f"Периметр квадрата: {square.perimeter():.2f}")

        # Треугольник внутри квадрата
        triangle_points = [
            Point(1, 1),
            Point(3, 1),
            Point(2, 3)
        ]
        triangle = ConvexPolygon(triangle_points)
        print(f"\nТреугольник: {triangle}")
        print(f"Площадь треугольника: {triangle.area():.2f}")
        print(f"Периметр треугольника: {triangle.perimeter():.2f}")

        # Пятиугольник для триангуляции
        pentagon_points = [
            Point(0, 0),
            Point(3, 0),
            Point(4, 2),
            Point(2, 4),
            Point(-1, 2)
        ]
        pentagon = ConvexPolygon(pentagon_points)
        print(f"\nПятиугольник: {pentagon}")
        print(f"Площадь пятиугольника: {pentagon.area():.2f}")
        print(f"Периметр пятиугольника: {pentagon.perimeter():.2f}")

        print(f"Квадрат пересекается с треугольником: {square.intersect(triangle)}")
        print(f"Квадрат содержит треугольник: {square.contains_polygon(triangle)}")

        triangles = pentagon.triangulate()
        print(f"Исходный пятиугольник разбит на {len(triangles)} треугольника:")
        print(triangles)
       


    except Exception as e:
        print(f"Ошибка: {e}")
