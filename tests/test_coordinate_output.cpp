#include "coordinate_output.h"

#include <iostream>
#include <limits>

void expect_close(const coordinates::Point &actual, const coordinates::Point &expected)
{
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(actual[i] - expected[i]) > 1e-8) {
            throw std::runtime_error("Unexpected transformed coordinate.");
        }
    }
}

template<class Action> void expect_rejected(Action action)
{
    try { action(); } catch (const std::runtime_error &) { return; }
    throw std::runtime_error("Invalid coordinate transform was accepted.");
}

int main()
{
    try {
        const double angle = 0.6, c = std::cos(angle), s = std::sin(angle);
        coordinates::FloorTransform transform{{{{1., 0., 0.}, {0., -c, -s}, {0., -s, c}}},
                                               {0., 2100., 0.}, {-1200., 0., 500.}};
        transform.validate();
        // A point 1017 mm above the floor, 300 mm right and 1800 mm forward.
        const coordinates::Point native{300., c * (2100. - 1017.) - s * 1800.,
                                              s * (2100. - 1017.) + c * 1800.};
        const auto floor = transform.to_floor(native);
        expect_close(floor, {300., 1017., 1800.});
        const auto virtual_aux = transform.to_virtual_aux_90(floor);
        expect_close(virtual_aux, {-1300., 1017., 1500.});
        expect_close({virtual_aux[2] - 1200., virtual_aux[1], -virtual_aux[0] + 500.}, floor);
        expect_close(transform.to_floor({0., 2100. * c, 2100. * s}), {0., 0., 0.});
        expect_rejected([&] { coordinates::validate_basis(transform.basis, false); });
        auto bad = transform;
        bad.basis[0][0] = 1.1;
        expect_rejected([&] { bad.validate(); });
        expect_rejected([&] { transform.to_floor({0., std::numeric_limits<double>::quiet_NaN(), 1.}); });
        std::cout << "Floor coordinate and virtual 90-degree tests passed.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
