# VS Codeでのコードブラウズ

## 解析構成

リポジトリのルートをVS Codeで開く。Microsoft C/C++拡張機能
(`ms-vscode.cpptools`)で補完、定義ジャンプ、参照検索を行う。
`.vscode/settings.json`で、このプロジェクトのIntelliSenseを有効にし、
STM32Cube clangdを無効にする。ユーザー全体の設定は変更しない。

`.vscode/c_cpp_properties.json`にはDebugとReleaseの2構成を用意する。
コンパイラは既存ビルドスクリプトと同じSTM32CubeCLT 1.21.0の
`arm-none-eabi-gcc.exe`を使用する。Cortex-M4、Thumb、FPv4-SP-D16、hard-floatを
指定し、標準ヘッダーと組み込みマクロはコンパイラから取得する。
独自の`M_PI`定義やツールチェーンの標準ヘッダーの手動指定は不要。

各Cファイルの実際のオプションはコンパイルデータベースを優先する。
アプリはDebug/ReleaseのMakefile、ブートローダーはBootloader/Makefileから
取得するため、ブートローダーにはアプリの`USE_HAL_DRIVER`や`DEBUG`を混入させない。
アプリとブートローダーの同名ヘッダーは、各ソースのインクルードパスで解決する。
アプリのフォールバック設定はGNU C17、ブートローダーの個別設定はGNU C11となる。

この仕組みはMicrosoftの[クロスコンパイル設定](https://code.visualstudio.com/docs/cpp/configure-intellisense-crosscompilation)
および[コンパイルデータベースの設定](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)に従う。

## 初回・Makefile変更後の更新

生成JSONは絶対パスを含むためGit管理から除外している。
別のチェックアウトやソース・ビルド設定変更後は、ルートから実行する。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\update_code_browse.ps1 -Configuration Debug
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\update_code_browse.ps1 -Configuration Release
```

`update_code_browse.ps1`はmakeの`-B -n`で全コンパイルコマンドを取得する。
コンパイルや基板への書き込みは行わない。アプリとブートローダーのCファイルを
`.vscode/compile_commands.Debug.json`または`compile_commands.Release.json`にまとめる。
新しいソースは、先にCubeIDEで対応するMakefileへ反映する必要がある。
ツールの場所が異なる場合は`-MakePath`と`-ToolchainBin`を指定し、
`c_cpp_properties.json`の`compilerPath`も同じコンパイラへ更新する。

VS Codeでは「タスク: タスクの実行」の`Refresh code browse (Debug)`または
`Refresh code browse (Release)`でも更新できる。
「C/C++: 構成の選択」でDebug/Releaseを切り替える。
Ctrl+Shift+BはDebugのビルドと解析設定更新を実行する。
Releaseとブートローダーはそれぞれのビルドタスクを選択する。

## 動作確認・トラブル時

設定反映後は「開発者: ウィンドウの再読み込み」を実行する。
`Core/Src/main.c`でHAL関数のF12、`Bootloader/Src/main.c`でブートローダー関数のF12、
Shift+F12による参照検索、ホバー表示・補完を確認する。

解析されない場合は「C/C++: 診断のログ」でコンパイラ、構成名、各ファイルの
インクルードパスを確認する。古い解析結果が残る場合は
「C/C++: IntelliSense データベースのリセット」を実行する。
STM32Cube clangdをこのワークスペースで再有効化すると解析エンジンが重複するため、
この構成では無効のまま使用する。
