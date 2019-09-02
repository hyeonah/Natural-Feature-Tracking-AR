#include "ChipsEninge.h"
#include "MarSystem/BasicFrame/MarApp.h"
#include "DirectX/Global/Global.h"

/*추가적으로 할 것*/
//Recognizer는 지금은 Camera를 움직이지만, 나중에는 객체를 움직이게 할 것임, 
//내 생각에는 카메라를 움직이면 여러 개의 객체를 트래킹할 수 없을 것 같음 (카메라 위치 기준 상대적으로 물체를 이동)
//Recognizer에서 카메라 파라미터 매개변수로 받아서 설정할 수 있게 수정
//MarApp Ar모드, Normal모드 선택 가능하게 설정 추가 해야함
//Trnasform 작성
//RenderComponenets 몽땅 작성
//Sensor 몽땅 작성
//RecognizerComponenets 몽땅 작성
//AudioManager 작성
//EventManager 작성 (충돌처리까지)
//GestureManager 작성
//VoiceManager 작성
//Interpreter 작성
//ETC 필요하면 작성

int WINAPI WinMain(HINSTANCE _hInstance, HINSTANCE _prevInstance, PSTR _cmdLine, int _showCmd)
{
#if defined(DEBUG) | defined(_DEBUG)
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
	
	MarFrame::MarApp marApp("DriectX", _hInstance);
	marApp.Run();

	return 0;
}