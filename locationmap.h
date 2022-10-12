#ifndef LOCATIONMAP_H
#define LOCATIONMAP_H
#include <iostream>
#include <vector>
#include <boost/qvm/vec.hpp>

typedef boost::qvm::vec<float,3> vec3;

using namespace std;

/// @brief Интерфейс матрицы расстояний
class IDistanceMatrix
{
public:
    /// Инициализация матрицы расстояний
    void virtual init()=0;

    /// Виртуальный деструктор
    virtual ~IDistanceMatrix()=0;
};

/// @brief Базовый класс матрицы расстояний
class DistanceMatrix:public IDistanceMatrix
{
public:
    /// Вектор точек будущей матрицы
    vector<vec3> matrix;
    void init()override;
     ~DistanceMatrix()override;
};

/// @brief Класс прямоугольной матрицы расстояний
/** @details
 *  <p>Матрица создается на базе вектора точек (x, y).
 *  <br>Значение <b>width</b> задает количество элементов матрицы по ширине.
 *  <br>Значение <b>height</b> задает количетво элементов матрицы по высоте.
 *  <br>Значение <b>step</b>(шаг) отвечает за расстояние от одной точки до другой.
 *  <br>Значение <b>center</b> задает номер элемента матрицы, который будет считаться точкой отсчета.
 *  <br>На рисунке представлена схема прямоугольной матрицы расстояний, с точкой отсчета в элементе под номером 12.
 *  <p>
 *  <img src="pic/DMQuadrAngle1.svg" alt="Схема прямоугольной матрицы расстояний" height="425px" width="450px" />
*/
class DMQuadrangle: public DistanceMatrix
{
public:
    /// Ширина матрицы
    int width;

    /// Высота матрицы
    int height;

    /// Точка отсчета в прямоугольной системе координат
    int center;

    /// Шаг элементов матрицы расстояний
    float step;

    /// Конструктор по умолчанию
    DMQuadrangle();

    /// Конструктор с параметрами
    DMQuadrangle(int width, int height, int center, float step);

    /// Конструктор копирования
    DMQuadrangle(const DMQuadrangle&o);

    void init() override;

    int GetI(float j) const
    {
        return (center - int(j / step) * width) / width;
    }

    int GetJ(float i) const
    {
        return (center + int(i/step)) % width;
    }

    ~DMQuadrangle()override;
};

class DistanceMatrixPrinter
{
public:
    template<class DistanceMatrix>
    void print(const DistanceMatrix& distance);
};

#endif // LOCATIONMAP_H