#pragma once

namespace stats {

template<class iterator_t>
inline typename iterator_t::value_type mean(const iterator_t& begin, const iterator_t& end)
{
    iterator_t::value_type r = 0;
    u32 count = 0;
    for (iterator_t it = begin; it != end; ++it, ++count)
        r += *it;
    return r / static_cast<iterator_t::value_type>(count);
}

template<class iterator_t>
inline typename iterator_t::value_type variance(const iterator_t& begin, const iterator_t& end, typename iterator_t::value_type precomputed_mean)
{
    iterator_t::value_type r = 0, temp;
    u32 count = 0;
    for (iterator_t it = begin; it != end; ++it, ++count)
    {
        temp = *it - precomputed_mean;
        r += temp * temp;
    }
    return r / static_cast<iterator_t::value_type>(count);
}
template<class iterator_t>
inline typename iterator_t::value_type variance(const iterator_t& begin, const iterator_t& end)
{
    return variance(begin, end, mean(begin, end));
}

}