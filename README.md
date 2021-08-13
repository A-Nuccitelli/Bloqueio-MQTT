<h1>Bloqueador de equipamento, por protocolo MQTT</h1>

<h3>Bibliotecas usadas</h3>

WifiManager v1.7.3 que incluí: (Utilizado para configurar o wifi pelo celular)

 *	FS
 *	ArduinoJson (Pacote de dados)
 *	DNSServer
 *	ESP8266Wifi
 *	ESP8266WEBSERVER (Server HTML)
 *	ESP8266WifiMulti
 *	LittleFS ou SPIFFS (Armazenamento na flash da config do wifi)

EEPROM (Armazenar os valores de variáveis importantes para o funcionamento)

PubSubClient (Biblioteca do MQTT)



<h3>Passo a passo de como foi elaborado o código</h3>

Primeiramente foi trabalhado por partes, cada função, depois unificados alguns exemplos e adaptados para a necessidade em questão.



Em sua estrutura, foi adicionado o wifimanager e modificado, em seguida a função eeprom e depois o mqtt.

Declarada todas as variáveis e void e depois no setup inserido os voids em sua ordem de importância para inicio.

Primeiro o void que inicia as variaveis e seus valores iniciais, depois o void da eeprom para atualizar o ultimo valor deixado no equipamento e em seguida o void tarefa, responsavel por contabilizar a hora do equipamento e tomar a decisão de quando efetuar o bloqueio através de seu setup.

Depois inicia o void trava ainda no setup, em seguida o wifi e por ultimo o mqtt.



Vale lembrar que todos trabalham de maneira independente passando pelo loop e não travando, sendo tratadas todas as situações de loop eterno.



no Loop, foi chamados os voids de função, tarefa, trava, verifica conexão wifi e mqtt e loop do mqtt, todos para analisar o estado atual e executar uma função se necessario.



<h3>Comandos setados no codigo</h3>

MQTT

 *	L 			(Liga a trava)
 *	D 			(Desliga a trava)
 *	R			(Reseta o valor de tempo corrido)
 *	S28A		(Setup o valor de tempo para correr 28 dias trabalhando 10 horas diários)
	*	S28C		(Setup o valor de tempo para correr 28 dias trabalhando 24 horas diários)
	*	S183		(Setup o valor de tempo para correr 183 dias trabalhando 24 horas diários)
	*	S365		(Setup o valor de tempo para correr 365 dias trabalhando 24 horas diários)

Código

 * EstadoSaida (Encaminha as mensagens de retorno por MQTT)
 * Tempo (Conta o tempo de funcionamento do equipamento)
 * Trava (Valor referencia do setup para a decisão de bloqueio)

Servidores MQTT

	* test.mosquitto.org
	* broker.emqx.io
	* anuccitelli.ddns.net:80

Regras para topico MQTT

* MQTTMKFenvio-reg01 (Reg01 equivalente ao cliente)
* MQTTMKFrecebe-reg01
* MQTTMKF-reg01