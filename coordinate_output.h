#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace coordinates {
using Point = std::array<double, 3>;
using Matrix = std::array<Point, 3>;

inline void validate_basis(const Matrix &matrix, bool allow_reflection)
{
    for (size_t row = 0; row < 3; ++row) {
        for (size_t other = 0; other < 3; ++other) {
            double dot = 0.0;
            for (size_t column = 0; column < 3; ++column) {
                if (!std::isfinite(matrix[row][column])) {
                    throw std::runtime_error("Coordinate basis must be finite.");
                }
                dot += matrix[row][column] * matrix[other][column];
            }
            if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1e-5) {
                throw std::runtime_error("Coordinate basis must be orthonormal.");
            }
        }
    }
    const auto &m = matrix;
    const double determinant =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (!allow_reflection && determinant < 0.0) {
        throw std::runtime_error("Physical camera rotation must preserve handedness.");
    }
}

inline void validate_point(const Point &point)
{
    for (double value : point) {
        if (!std::isfinite(value)) {
            throw std::runtime_error("Coordinate point must be finite.");
        }
    }
}

struct FloorTransform {
    Matrix basis{};
    Point translation_mm{};
    Point virtual_aux_origin_mm{};

    void validate() const
    {
        validate_basis(basis, true);
        validate_point(translation_mm);
        validate_point(virtual_aux_origin_mm);
        if (translation_mm[1] <= 0.0 || std::abs(virtual_aux_origin_mm[1]) > 1e-5) {
            throw std::runtime_error("Camera height must be positive and virtual AUX origin must be on the floor.");
        }
    }

    Point to_floor(const Point &native_base) const
    {
        validate_point(native_base);
        Point result = translation_mm;
        for (size_t row = 0; row < 3; ++row) {
            for (size_t column = 0; column < 3; ++column) {
                result[row] += basis[row][column] * native_base[column];
            }
        }
        return result;
    }

    Point to_virtual_aux_90(const Point &floor) const
    {
        // Virtual AUX forward points along BASE floor +X, exactly 90 degrees.
        return {virtual_aux_origin_mm[2] - floor[2], floor[1], floor[0] - virtual_aux_origin_mm[0]};
    }
};
} // namespace coordinates
