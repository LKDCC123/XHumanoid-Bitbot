/****************************************************************************
MIT License

Copyright (c) 2024 zishun zhou

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

#pragma once
#ifndef CONTACT_ESTIMATOR_HPP
#define CONTACT_ESTIMATOR_HPP

#include <array>
#include <iostream>
#include "libdef.h"

namespace zzs
{
    template <typename SCALAR>
    class INEKF_API BipedalContactEstimator
    {
    public:
        using Ptr=std::unique_ptr<BipedalContactEstimator<SCALAR>>;

    public:
        BipedalContactEstimator(const SCALAR contact_force_threshold,
            const SCALAR debounce_delay,
            const SCALAR dt
        ) :last_check_contact(0)
            , contact_thres(contact_force_threshold)
            , last_contact_status({ false,false })
            , debounce_delay_cnt(static_cast<size_t>(debounce_delay) / dt)
        {
        }

        std::array<bool, 2> operator() (const std::array<SCALAR, 2>& contact_force)
        {
            if (last_check_contact == 0) //稳定状态
            {
                std::array<bool, 2> cmp_contact;
                cmp_contact[0] = contact_force[0] >= contact_force[1];
                cmp_contact[1] = contact_force[1] > contact_force[0];

                if (contact_force[0] > contact_thres && contact_force[1] > contact_thres) //双腿都大于阈值，直接判定双腿站立
                {
                    //std::cout << "dual stand" << std::endl;
                    last_contact_status = { true,true };
                }
                else if (cmp_contact[0] != last_contact_status[0] || cmp_contact[1] != last_contact_status[1])
                {
                    last_check_contact = debounce_delay_cnt;
                    last_contact_status = cmp_contact;
                    //std::cout << "status changed\n";
                    //std::cout << "left=" << last_contact_status[0] << " right=" << last_contact_status[1] << std::endl;
                }
                else
                {
                    last_contact_status = cmp_contact;
                    //std::cout << "status keep\n";
                    //std::cout << "left=" << last_contact_status[0] << " right=" << last_contact_status[1] << std::endl;
                }
            }
            else //不稳定状态
            {
                //刚切换状态之后的一段时间内不检测了
            }

            --last_check_contact;
            last_check_contact = std::max(last_check_contact, static_cast<int>(0));
            //std::cout << "last_check_contact=" << last_check_contact << std::endl;
            return last_contact_status;
        }

    private:
        std::array<bool, 2> last_contact_status;
        const size_t debounce_delay_cnt;
        const SCALAR contact_thres;
        int last_check_contact;
    };

    using BipedalContactEstimatorf = BipedalContactEstimator<float>;
    using BipedalContactEstimatord = BipedalContactEstimator<double>;
};

#endif // CONTACT_ESTIMATOR_HPP