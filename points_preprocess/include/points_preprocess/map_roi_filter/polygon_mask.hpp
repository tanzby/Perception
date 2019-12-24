#pragma once
#include <algorithm>
#include <limits>
#include <vector>
#include <glog/logging.h>

#include <Eigen/StdVector>

#include "points_preprocess/map_roi_filter/bitmap_2d.hpp"
#include "polygon_scan_converter.hpp"

namespace points_preprocess {

    template <typename T>
    bool DrawPolygonMask(const typename PolygonScanConverter<T>::Polygon& polygon,
                         Bitmap2D* bitmap, const double extend_dist = 0.0,
                         const bool no_edge_table = false);

    template <typename T>
    bool DrawPolygonsMask(
            const std::vector<typename PolygonScanConverter<T>::Polygon>& polygons,
            Bitmap2D* bitmap, const double extend_dist = 0.0,
            const bool no_edge_table = false);

    template <typename T>
    bool DrawPolygonMask(const typename PolygonScanConverter<T>::Polygon& polygon,
                         Bitmap2D* bitmap, const double extend_dist,
                         const bool no_edge_table) {
        typedef typename PolygonScanConverter<T>::IntervalIn IntervalIn;
        typedef typename PolygonScanConverter<T>::IntervalOut IntervalOut;
        typedef typename PolygonScanConverter<T>::DirectionMajor PolyDirMajor;
        assert(!bitmap->Empty());
        Eigen::Vector2d poly_min_p, poly_max_p;
        poly_min_p.setConstant(std::numeric_limits<double>::max());
        poly_max_p = -poly_min_p;
        for (const auto& pt : polygon) {
            poly_min_p.x() = std::min(pt.x(), poly_min_p.x());
            poly_min_p.y() = std::min(pt.y(), poly_min_p.y());

            poly_max_p.x() = std::max(pt.x(), poly_max_p.x());
            poly_max_p.y() = std::max(pt.y(), poly_max_p.y());
        }
        CHECK_GT(poly_max_p.x(), poly_min_p.x()) << " with " << polygon.size() <<" points";
        CHECK_GT(poly_max_p.y(), poly_min_p.y()) << " with " << polygon.size() <<" points";;

        const Eigen::Vector2d& bitmap_min_range = bitmap->min_range();
        const Eigen::Vector2d& bitmap_max_range = bitmap->max_range();
        const Eigen::Vector2d& cell_size = bitmap->cell_size();
        // TODO(...) maybe confused
        int major_dir = static_cast<int>(bitmap->dir_major());
        int op_major_dir = static_cast<int>(bitmap->op_dir_major());

        // check major x range
        IntervalIn valid_range;
        valid_range.first =
                std::max(poly_min_p[major_dir], bitmap_min_range[major_dir]);
        valid_range.second =
                std::min(poly_max_p[major_dir], bitmap_max_range[major_dir]);

        // for numerical stability
        valid_range.first =
                (static_cast<int>((valid_range.first - bitmap_min_range[major_dir]) /
                                  cell_size[major_dir]) +
                 0.5) *
                cell_size[major_dir] +
                bitmap_min_range[major_dir];

        if (valid_range.second < valid_range.first + cell_size[major_dir]) {
            ROS_ERROR_STREAM("Invalid range: " << valid_range.first << " " << valid_range.second
                  << ". polygon major directory range: " << poly_min_p[major_dir] << " "
                  << poly_max_p[major_dir] << ". cell size: " << cell_size[major_dir]);
            return true;
        }

        // start calculating intervals of scans
        PolygonScanConverter<T> poly_scan_cvter;
        poly_scan_cvter.Init(polygon);
        std::vector<std::vector<IntervalOut>> scans_intervals;
        if (no_edge_table) {
            size_t scans_size =
                    static_cast<size_t>((valid_range.second - valid_range.first) /
                                        cell_size[major_dir]);
            scans_intervals.resize(scans_size);
            for (size_t i = 0; i < scans_size; ++i) {
                double scan_loc = valid_range.first + i * cell_size[major_dir];
                poly_scan_cvter.ScanCvt(scan_loc, static_cast<PolyDirMajor>(major_dir),
                                        &(scans_intervals[i]));
            }
        } else {
            poly_scan_cvter.ScansCvt(valid_range, static_cast<PolyDirMajor>(major_dir),
                                     cell_size[major_dir], &(scans_intervals));
        }
        // start to draw
        double x = valid_range.first;
        for (size_t i = 0; i < scans_intervals.size();
             x += cell_size[major_dir], ++i) {
            for (auto scan_interval : scans_intervals[i]) {
                if (scan_interval.first > scan_interval.second) {
                    ROS_ERROR_STREAM("The input polygon is illegal(complex polygon)");
                    return false;
                }

                // extend
                scan_interval.first -= extend_dist;
                scan_interval.second += extend_dist;

                IntervalOut valid_y_range;
                valid_y_range.first =
                        std::max(bitmap_min_range[op_major_dir], scan_interval.first);
                valid_y_range.second =
                        std::min(bitmap_max_range[op_major_dir], scan_interval.second);
                if (valid_y_range.first > valid_y_range.second) {
                    continue;
                }
                bitmap->Set(x, valid_y_range.first, valid_y_range.second);
            }
        }
        return true;
    }

    template <typename T>
    bool DrawPolygonsMask(
            const std::vector<typename PolygonScanConverter<T>::Polygon>& polygons,
            Bitmap2D* bitmap, const double extend_dist, const bool no_edge_table) {
        for (const auto& polygon : polygons) {
            bool flag = DrawPolygonMask<T>(polygon, bitmap, extend_dist, no_edge_table);
            if (!flag) {
                return false;
            }
        }
        return true;
    }

}

