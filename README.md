# Detector-de-Queda

Este projeto é um protótipo de sensor de queda com ESP32, pensado para ajudar a monitorar possíveis quedas de pessoas idosas. Ele envia alerta por Telegram e mostra um painel de status em um navegador.

---

## O que este projeto faz

Imagine um aparelho que vigia quedas de idosos e avisa um familiar ou cuidador pela internet quando detecta um tombo.

Ele faz isso assim:

- lê movimentos do sensor MPU6050
- decide se o movimento parece uma queda
- toca um buzzer para avisar localmente
- envia uma mensagem para o Telegram
- mostra um painel de monitoramento no navegador

---

## Como o sistema está dividido

O código roda em duas partes principais ao mesmo tempo:

1. **Detecção de queda**
2. **Rede e painel web**
3. **Monitor de saúde do equipamento**

Isso permite que ele continue detectando quedas mesmo quando está enviando alerta para a internet.

```
[ESP32 Core 0] ---> Detecta queda     
[ESP32 Core 1] ---> Envia alerta e atualiza painel
[Task independente] -> Mede temperatura, memória e uso da CPU
```

---

## O que acontece quando há uma queda

O sensor faz três verificações simples:

- ele detecta se o dispositivo está em **queda livre** (como se estivesse caindo)
- ele detecta se houve um **choque forte** logo depois
- ele também aceita um choque muito forte sem queda livre, caso seja um impacto brusco

Se uma queda é confirmada:

- o buzzer toca por alguns segundos
- o alerta é enviado para o Telegram
- o painel web mostra que uma queda foi registrada

---

## O painel web

O projeto inclui um site simples que mostra em tempo real:

- quantas quedas já foram detectadas
- quantos alertas foram enviados
- qual é o IP do ESP32
- qual é o nível do sinal WiFi
- temperatura do chip
- uso de memória e espaço em disco
- uso da CPU das tarefas
- latência do alerta

O painel também tem botão para expandir e mostrar mais informações.

### O que você verá no painel

- **Eventos**: quedas e alertas
- **Rede WiFi**: IP, nome da rede e força do sinal
- **Sistema**: modelo do chip, temperatura e tempo ligado
- **Desempenho**: quanto tempo cada tarefa demora para rodar
- **Memória**: quanto espaço ainda está livre

---

## Passo a passo simples de uso

1. Grave o código em um ESP32.
2. Ao ligar pela primeira vez ou depois de reset duplo, o ESP32 virará uma rede WiFi chamada `Fall_detector`.
3. Conecte no WiFi e abra o portal de configuração.
4. Informe:
   - seu nome
   - token do bot do Telegram
   - chat ID do Telegram
5. O dispositivo se conecta à sua rede WiFi.
6. Abra o painel no navegador usando o endereço mostrado no monitor serial.

---

## O que cada parte faz

### 1) Detectar queda

- roda em um dos núcleos do ESP32
- lê o sensor MPU6050 a cada 20 ms
- calcula a intensidade total do movimento
- decide se é queda livre ou impacto
- dispara um alerta quando detecta uma queda

### 2) Enviar alerta para o Telegram

- roda no outro núcleo do ESP32
- mantém o painel web disponível
- quando recebe aviso de queda, manda mensagem para o Telegram
- mede quanto tempo demora para enviar

### 3) Monitorar o equipamento

- roda separadamente
- a cada segundo mede:
  - temperatura do chip
  - força do sinal WiFi
  - memória livre
  - uso da CPU
- mantém esses dados prontos para o painel web

---

## O que você vê no navegador

O painel mostra um gráfico simples de barras com:

- uso de CPU das tarefas
- quantidade de memória livre
- latência do aviso

Basta abrir a página e ela atualiza sozinha a cada segundo.

### Uso da página

- `Expandir/Recolher`: mostra mais ou menos informações
- `Simular queda (teste)`: faz o sistema agir como se tivesse detectado uma queda

---

## O que está dentro do código

O arquivo `Project/detection.ino` contém:

- a configuração do WiFi e do bot do Telegram
- o código que lê o sensor MPU6050
- a lógica da detecção de queda
- o servidor web com o painel
- a parte que salva as configurações no armazenamento interno
- as três tarefas que rodam em paralelo

---

## Por que isso é útil

- permite acompanhar quedas em tempo real
- avisa alguém automaticamente pelo Telegram
- mantém um painel de controle que mostra o estado do aparelho
- ajuda a entender se o dispositivo está funcionando bem enquanto monitora movimentações

---

## Se quiser testar sem cair de verdade

Use o botão `Simular queda (teste)` no painel.

Ele faz o mesmo que uma queda real, sem precisar mexer no aparelho.

---

## Aviso rápido

- é um protótipo
- o sistema funciona melhor em um ambiente de teste
- o alerta do Telegram depende de internet funcionando

---

## Diagrama de funcionamento

1. **Ligou o ESP32**
2. **Entrou em modo de configuração se necessário**
3. **Conectou ao WiFi**
4. **Começou a ler o sensor**
5. **Detectou possível queda**
6. **Acionou buzzer + mandou Telegram**
7. **Atualizou o painel no navegador**

---

## Palavras-chave fáceis

- `ESP32` = placa que roda o projeto
- `MPU6050` = sensor que sente movimento
- `buzzer` = o som de alerta
- `Telegram` = onde chega a mensagem
- `dashboard` = painel web para ver os dados
