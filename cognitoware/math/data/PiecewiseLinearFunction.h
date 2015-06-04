/*
 * PiecewiseLinearFunction.h
 *
 *  Copyright (c) 2015, Norman Alan Oursland
 *  All rights reserved.
 */

#ifndef MATH_DATA_PIECEWISELINEARFUNCTION_H_
#define MATH_DATA_PIECEWISELINEARFUNCTION_H_

#include <list>
#include <vector>
#include <functional>

namespace cognitoware {
namespace math {
namespace data {

class PiecewiseLinearFunction
final {
  public:
    typedef std::function<double(double)> Fn;

    PiecewiseLinearFunction() {
    }
    PiecewiseLinearFunction(Fn fn, double min, double max, int n,
                            double upperError, double lowerError);
    PiecewiseLinearFunction(const PiecewiseLinearFunction&& that);
    PiecewiseLinearFunction& operator=(PiecewiseLinearFunction&& that);

    double Evaluate(double x) const;
    PiecewiseLinearFunction PdfToCdf();
  private:
    static std::list<std::pair<double, double>> BuildFn(Fn fn, double min,
                                                        double max, int n,
                                                        double upperError,
                                                        double lowerError);
    static bool ExpandResults(Fn fn, double upperBound,
                              std::list<std::pair<double, double>>* p_pts);
    static bool PruneResults(double lowerBound,
                             std::list<std::pair<double, double>>* p_pts);

    std::vector<double> domain_;
    std::vector<double> range_;
  };

  }  // namespace data
  }  // namespace math
  }  // namespace cognitoware

#endif /* MATH_DATA_PIECEWISELINEARFUNCTION_H_ */
