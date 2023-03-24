# sterowniki_robotow

W ramach projektu realizowany będzie program wykrywania komend głosowych, oraz odtwarzania plików audio za pomocą mikrokontrolera STM32L476 Discovery.

Nasłuchiwanie i komunikacja dźwiękowa odbywać się będzie za pomocą audio DAC'a. Komendy głosowe będą rozpoznawane poprzez porównywanie widm dźwięków przesłanych do mikrokontrolera z próbkami wcześniej do niego wgranymi. Dostępnych będzie około 6 komend odpowiadających za poszczególne akcje przy odtwarzaniu dźwięków. Przetwarzanie dźwięku wejściowego odbywać się będzie za pomocą szybkiej transformaty Fourier'a.
