# NS-3 LR-WPAN CSMA/CA TESTBED

## 1. 개요
- `LrWpanCsmaCa`: `LrWpanCsmaCaCommon` 추상 클래스를 상속받는 클래스로 구성
- `LrWpanCsmaCaCommon` 클래스를 구현하는 다양한 기법을 구현 가능

## 2. 주요 기능
- 구현체별 `LrWpanMac`에 `CsmaOption`을 지정하고, 해당 옵션에서만 필요한 동작을 추가할 수 있음
- [CSMA/CA - NOBA](https://ieeexplore.ieee.org/document/9091031) 메커니즘 구현: `lr-wpan-csma-noba`
- (예정) [CSMA/CA - SW-NOBA]() 메커니즘 구현: `lr-wpan-csma-sw-noba`

## 3. Installation
- 기반 코드는 ns-3.43 lr-wpan 모듈을 기반으로 작성되었습니다.

```
$ git clone github.com/gmelan/lr-wpan-csma-testbed lr-wpan
$ rm -r path/to/ns3/src/lr-wpan # THIS WILL DELETE PREVIOUS lr-wpan MODULE
$ mv lr-wpan path/to/ns3/src/
```
