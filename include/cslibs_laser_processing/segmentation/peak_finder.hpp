    #ifndef PEAK_FINDER_HPP
#define PEAK_FINDER_HPP

/// SYSTEM
#include <vector>
#include <boost/shared_ptr.hpp>

template<typename Tp>
class PeakFinder {
public:
    typedef boost::shared_ptr<PeakFinder> Ptr;

    virtual ~PeakFinder()
    {
    }

    void findPeaks(const std::vector<Tp> ranges, std::vector<unsigned int> &indexes)
    {
        /// PREPARE
        indexes.clear();

        /// COMPUTE
        typename std::vector<Tp>::const_iterator prev = ranges.begin();
        typename std::vector<Tp>::const_iterator curr = prev + margin_;
        typename std::vector<Tp>::const_iterator next = curr + margin_;

        unsigned int i = 0;
        for( ; next != ranges.end() ; ++prev, ++curr, ++next, ++i) {
            if(*curr > min_range_ && *curr < max_range_ && isPeak(prev, curr, next)) {
                indexes.push_back(i);
            }
        }
    }

    void setMinRange(const Tp min_range)
    {
        min_range_ = min_range;
    }

    void setMaxRange(const Tp max_range)
    {
        max_range_ = max_range;
    }

    void setMinDistance(const Tp min_distance)
    {
        min_distance_ = min_distance;
    }

    void setMargin(const unsigned int margin = 1) {
        margin_ = margin;
    }

protected:
    Tp           min_distance_;
    Tp           min_range_;
    Tp           max_range_;
    unsigned int margin_;

    PeakFinder(const Tp min_distance, const Tp min_range, const Tp max_range) :
        min_distance_(min_distance),
        min_range_(min_range),
        max_range_(max_range)
    {
    }

    virtual bool isPeak(const typename std::vector<Tp>::const_iterator prev,
                        const typename std::vector<Tp>::const_iterator curr,
                        const typename std::vector<Tp>::const_iterator next) = 0;

    bool isMax(const typename std::vector<Tp>::const_iterator prev,
               const typename std::vector<Tp>::const_iterator curr,
               const typename std::vector<Tp>::const_iterator next)
    {
        Tp distance_prev = *curr - *prev;
        Tp distance_next = *curr - *next;

        return distance_next > min_distance_ && distance_prev > min_distance_;
    }

    bool isMin(const typename std::vector<Tp>::const_iterator prev,
               const typename std::vector<Tp>::const_iterator curr,
               const typename std::vector<Tp>::const_iterator next)
    {
        Tp distance_prev =  *prev - *curr;
        Tp distance_next =  *next - *curr;

        return distance_next > min_distance_ && distance_prev > min_distance_;
    }
};

template<typename Tp>
class MaxFinder : public PeakFinder<Tp> {
public:
    MaxFinder(const Tp min_distance, const Tp min_range, const Tp max_range) :
        PeakFinder<Tp>(min_distance, min_range, max_range)
    {
    }

protected:
    bool isPeak(const typename std::vector<Tp>::const_iterator prev,
                const typename std::vector<Tp>::const_iterator curr,
                const typename std::vector<Tp>::const_iterator next)
    {
        return this->isMax(prev, curr, next);
    }
};

template<typename Tp>
class MinFinder : public PeakFinder<Tp> {
public:
    MinFinder(const Tp min_distance, const Tp min_range, const Tp max_range) :
        PeakFinder<Tp>(min_distance, min_range, max_range)
    {
    }

protected:
    bool isPeak(const typename std::vector<Tp>::const_iterator prev,
                const typename std::vector<Tp>::const_iterator curr,
                const typename std::vector<Tp>::const_iterator next)
    {
        return this->isMin(prev, curr, next);
    }
};


template<typename Tp>
class MinMaxFinder : public PeakFinder<Tp> {
public:
     MinMaxFinder(const Tp min_distance, const Tp min_range, const Tp max_range) :
         PeakFinder<Tp>(min_distance, min_range, max_range)
     {
     }

protected:
       bool isPeak(const typename std::vector<Tp>::const_iterator prev,
                   const typename std::vector<Tp>::const_iterator curr,
                   const typename std::vector<Tp>::const_iterator next)
       {
           return this->isMin(prev, curr, next) || this->isMax(prev, curr, next);
       }
};



#endif // PEAK_FINDER_HPP
