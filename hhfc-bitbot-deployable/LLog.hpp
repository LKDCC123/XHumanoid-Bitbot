// LLog.hpp
// -- Version 0.0, Feb. 17, 2022
// Copyright (C) 2021-2022, Qingqing Li (liqingmuguang@163.com).
//
// This software may be modified and distributed under the terms
// of the MIT license.  See the LICENSE file for details.

#pragma once
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace lee
{
    namespace blocks
    {
        template <typename _T = double>
        class LLog
        {
        public:
            LLog()
            {
                this->CurrentDataIndex = 0;
            };
            inline void startLog()
            {
                this->CurrentDataIndex = 0;
            };
            void addLog(const _T& _SingleData, const char* _Name = "")
            {
                this->CurrentDataIndex++;
                if ((int)this->DataName.size() < this->CurrentDataIndex)
                {
                    this->DataName.push_back(_Name);
                    if ((int)this->DataList.size() < this->CurrentDataIndex)
                    {
                        this->DataList.push_back({ _SingleData });
                        return;
                    }
                }
                this->DataList[this->CurrentDataIndex - 1].push_back(_SingleData);
            };
            void saveLog(const char* _FileName)
            {
                this->FileStream.open(_FileName, std::fstream::out);
                for (auto i : this->DataName)
                    this->FileStream << i << ",";
                this->FileStream << std::endl;
                for (int i = 0; i < this->DataList[0].size(); i++)
                {
                    for (int j = 0; j < this->DataName.size(); j++)
                    {
                        this->FileStream << std::setprecision(12) << this->DataList[j][i] << ", ";
                    }
                    this->FileStream << std::endl;
                }
                this->FileStream.close();
            };

            inline auto& getDataList() { return this->DataList; };
            inline auto& getNameList() { return this->DataName; };
            inline auto getIndex(const char* _Str)
            {
                auto tar_iter = std::find(this->DataName.begin(), this->DataName.end(), _Str);
                return std::distance(this->DataName.begin(), tar_iter);
            };
            inline auto& getData(const char* _Str)
            {
                return this->getDataList()[this->getIndex(_Str)];
            };

            void initMemory(const unsigned int& _Kinds, const unsigned int& _Num)
            {
                this->DataList.resize(_Kinds);
                for (int i = 0; i < _Kinds; i++)
                {
                    this->DataList[i].reserve(_Num);
                }
            };

            void checkMemory()
            {
                std::cout << "Logger Memory Check:" << std::endl;
                for (int i = 0; i < this->DataList.size(); i++)
                {
                    std::cout << "Data " << i << " Capacity: " << this->DataList[i].capacity() << std::endl;
                }
            };

        protected:
            std::vector<std::vector<_T>> DataList;
            std::vector<std::string> DataName;
            int CurrentDataIndex;
            std::fstream FileStream;
        };
    }
}